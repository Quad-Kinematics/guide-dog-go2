// 3D point cloud map from the Go2's built-in L1 LiDAR, for viewing only
// (CloudCompare, Open3D, RViz). Navigation does not use it.
//
// /utlidar/cloud_deskewed is already motion-compensated by the robot and in
// the robot's odom frame, the same odom as /utlidar/robot_odom (which
// odom_to_tf publishes as odom -> base_link). So no deskewing or SLAM here:
// each cloud is moved to target_frame (map while AMCL runs, which also
// corrects odometry drift; odom without it) and averaged into a voxel grid.
// ~/save writes the grid to <output_dir>/<map_name>.<file_format>, and so
// does Ctrl-C (save_on_exit).

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

// Sparse voxel grid. Each cell keeps the mean position and intensity of the
// points that fell in it, and how many did (min_hits drops sparse cells).
class VoxelGrid {
public:
  struct Point { float x, y, z, intensity; };
  static_assert(sizeof(Point) == 16, "Point is copied straight into PointCloud2 and files");

  explicit VoxelGrid(double size) : size_(size) {}

  void add(double x, double y, double z, float intensity) {
    const int64_t ix = static_cast<int64_t>(std::floor(x / size_));
    const int64_t iy = static_cast<int64_t>(std::floor(y / size_));
    const int64_t iz = static_cast<int64_t>(std::floor(z / size_));
    // Sum the offsets inside the cell, not the coordinates: float sums of
    // thousands of hits at 30 m would lose centimetres.
    Cell & c = cells_[pack(ix, iy, iz)];
    c.sx += static_cast<float>(x - ix * size_);
    c.sy += static_cast<float>(y - iy * size_);
    c.sz += static_cast<float>(z - iz * size_);
    c.si += intensity;
    ++c.n;
  }

  std::vector<Point> points(int64_t min_hits) const {
    std::vector<Point> out;
    out.reserve(cells_.size());
    for (const auto & [key, c] : cells_) {
      if (c.n < min_hits) {
        continue;
      }
      const float n = static_cast<float>(c.n);
      out.push_back({
        static_cast<float>(unpack(key, 42) * size_ + c.sx / n),
        static_cast<float>(unpack(key, 21) * size_ + c.sy / n),
        static_cast<float>(unpack(key, 0) * size_ + c.sz / n),
        c.si / n});
    }
    return out;
  }

  size_t size() const { return cells_.size(); }
  void clear() { cells_.clear(); }

private:
  struct Cell { float sx = 0, sy = 0, sz = 0, si = 0; uint32_t n = 0; };

  // 21 bits per axis: +-2^20 cells, +-52 km at 5 cm
  static constexpr int64_t kOffset = int64_t{1} << 20;
  static constexpr uint64_t kMask = (uint64_t{1} << 21) - 1;

  static uint64_t pack(int64_t ix, int64_t iy, int64_t iz) {
    return ((static_cast<uint64_t>(ix + kOffset) & kMask) << 42) |
           ((static_cast<uint64_t>(iy + kOffset) & kMask) << 21) |
           (static_cast<uint64_t>(iz + kOffset) & kMask);
  }
  static int64_t unpack(uint64_t key, int shift) {
    return static_cast<int64_t>((key >> shift) & kMask) - kOffset;
  }

  double size_;
  std::unordered_map<uint64_t, Cell> cells_;
};

// Byte offset of a FLOAT32 field, or -1 if the cloud has none
static int float_field(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name) {
  for (const auto & f : cloud.fields) {
    if (f.name == name && f.datatype == sensor_msgs::msg::PointField::FLOAT32) {
      return static_cast<int>(f.offset);
    }
  }
  return -1;
}

static float read_float(const uint8_t * point, int offset) {
  float v;
  std::memcpy(&v, point + offset, sizeof(v));
  return v;
}

static std::string expand_home(const std::string & path) {
  const char * home = std::getenv("HOME");
  if (home && (path == "~" || path.rfind("~/", 0) == 0)) {
    return home + path.substr(1);
  }
  return path;
}

class CloudMapper : public rclcpp::Node {
public:
  CloudMapper()
  : Node("cloud_mapper")
  {
    // Params
    target_frame_ = this->declare_parameter<std::string>("target_frame", "map");
    const double voxel_size = this->declare_parameter<double>("voxel_size", 0.05);
    // Points farther than this from the robot are dropped (0 keeps all). The
    // L1 reaches ~22 m, but far points are sparse and hit walls at grazing
    // angles; walking past gives a closer look.
    max_range_ = this->declare_parameter<double>("max_range", 15.0);
    // Coarser copy of the map on /cloud_map for a live look at the coverage.
    // Kept small because point clouds through rosbridge on Foxy often fail.
    // publish_period 0 turns it off.
    const double preview_voxel_size = this->declare_parameter<double>("preview_voxel_size", 0.15);
    const double publish_period = this->declare_parameter<double>("publish_period", 5.0);
    save_on_exit_ = this->declare_parameter<bool>("save_on_exit", true);
    // Read at save time, so they can be changed with ros2 param set.
    // min_hits: drop cells hit fewer times (noise, people walking past).
    // map_name: empty gives cloud_map_<date>_<time>.
    this->declare_parameter<int>("min_hits", 1);
    this->declare_parameter<std::string>("output_dir", "~/maps_3d");
    this->declare_parameter<std::string>("map_name", "");
    this->declare_parameter<std::string>("file_format", "pcd");

    grid_ = std::make_unique<VoxelGrid>(voxel_size);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // Best effort: losing a cloud costs nothing, and the robot's own
    // publishers should never wait on this node.
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/utlidar/robot_odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { odom_ = msg; });
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/utlidar/cloud_deskewed", rclcpp::SensorDataQoS(),
      std::bind(&CloudMapper::on_cloud, this, std::placeholders::_1));

    if (publish_period > 0.0) {
      preview_ = std::make_unique<VoxelGrid>(preview_voxel_size);
      preview_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/cloud_map", rclcpp::QoS(1).transient_local());
      preview_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(publish_period),
        std::bind(&CloudMapper::publish_preview, this));
    }

    status_timer_ = this->create_wall_timer(10s, std::bind(&CloudMapper::log_status, this));

    save_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "~/save",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        res->success = save(res->message);
      });
    reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "~/reset",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        grid_->clear();
        if (preview_) {
          preview_->clear();
        }
        dirty_ = false;
        res->success = true;
        res->message = "map cleared";
        RCLCPP_INFO(this->get_logger(), "Map cleared");
      });

    RCLCPP_INFO(this->get_logger(),
      "Mapping /utlidar/cloud_deskewed into %s: voxel %.3f m, max_range %.1f m, preview %s",
      target_frame_.c_str(), voxel_size, max_range_,
      preview_ ? "on /cloud_map" : "off");
  }

  // Ctrl-C: save unless nothing was added since the last save
  void save_on_exit() {
    if (!save_on_exit_ || !dirty_) {
      return;
    }
    std::string result;
    save(result);
  }

private:
  void on_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    const int off_x = float_field(*msg, "x");
    const int off_y = float_field(*msg, "y");
    const int off_z = float_field(*msg, "z");
    const int off_i = float_field(*msg, "intensity");
    const size_t n = static_cast<size_t>(msg->width) * msg->height;
    if (off_x < 0 || off_y < 0 || off_z < 0 || msg->data.size() < n * msg->point_step) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Skipping cloud without float32 x/y/z or with short data");
      return;
    }

    tf2::Transform to_target;
    to_target.setIdentity();
    if (msg->header.frame_id != target_frame_) {
      geometry_msgs::msg::TransformStamped t;
      try {
        // Latest transform, not the one at header.stamp: the L1 stamps clouds
        // with the robot's clock, ~11 min behind the Jetson clock TF uses.
        // That is fine only because the cloud is already in odom and
        // map -> odom (AMCL) changes slowly. A cloud in a sensor frame would
        // need the transform at its capture time.
        t = tf_buffer_->lookupTransform(target_frame_, msg->header.frame_id, tf2::TimePointZero);
      } catch (const tf2::TransformException & e) {
        ++dropped_clouds_;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "No transform %s -> %s, skipping clouds (%s). Without AMCL use target_frame:=odom.",
          msg->header.frame_id.c_str(), target_frame_.c_str(), e.what());
        return;
      }
      const auto & tr = t.transform.translation;
      const auto & q = t.transform.rotation;
      to_target = tf2::Transform(tf2::Quaternion(q.x, q.y, q.z, q.w), tf2::Vector3(tr.x, tr.y, tr.z));
    }

    // Robot position for max_range, from /utlidar/robot_odom (same odom frame
    // and clock as the cloud)
    bool use_range = max_range_ > 0.0;
    double rx = 0.0, ry = 0.0, rz = 0.0;
    if (use_range) {
      if (!odom_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "Waiting for /utlidar/robot_odom (needed for max_range)");
        return;
      }
      if (odom_->header.frame_id != msg->header.frame_id) {
        RCLCPP_WARN_ONCE(this->get_logger(),
          "Odometry frame %s differs from cloud frame %s, max_range disabled",
          odom_->header.frame_id.c_str(), msg->header.frame_id.c_str());
        use_range = false;
      }
      rx = odom_->pose.pose.position.x;
      ry = odom_->pose.pose.position.y;
      rz = odom_->pose.pose.position.z;
    }
    const double max_range_sq = max_range_ * max_range_;

    size_t added = 0;
    for (size_t i = 0; i < n; ++i) {
      const uint8_t * p = &msg->data[i * msg->point_step];
      const float x = read_float(p, off_x);
      const float y = read_float(p, off_y);
      const float z = read_float(p, off_z);
      // The L1 pads every message to a fixed size (~10.4k points) with exact
      // zeros; only ~450 points per message are real.
      if (x == 0.0f && y == 0.0f && z == 0.0f) {
        continue;
      }
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      if (use_range) {
        const double dx = x - rx, dy = y - ry, dz = z - rz;
        if (dx * dx + dy * dy + dz * dz > max_range_sq) {
          continue;
        }
      }
      const float intensity = off_i >= 0 ? read_float(p, off_i) : 0.0f;
      const tf2::Vector3 v = to_target * tf2::Vector3(x, y, z);
      grid_->add(v.x(), v.y(), v.z(), intensity);
      if (preview_) {
        preview_->add(v.x(), v.y(), v.z(), intensity);
      }
      ++added;
    }

    ++clouds_;
    points_since_status_ += added;
    if (added > 0) {
      dirty_ = true;
    }
  }

  void publish_preview() {
    if (preview_pub_->get_subscription_count() == 0) {
      return;
    }
    const auto pts = preview_->points(1);
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = target_frame_;
    msg.header.stamp = this->now();
    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2Fields(4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(pts.size());
    std::memcpy(msg.data.data(), pts.data(), pts.size() * sizeof(VoxelGrid::Point));
    preview_pub_->publish(msg);
  }

  void log_status() {
    RCLCPP_INFO(this->get_logger(),
      "%zu voxels from %zu clouds (%.0f points/s in the last 10 s), %zu clouds skipped without TF",
      grid_->size(), clouds_, points_since_status_ / 10.0, dropped_clouds_);
    points_since_status_ = 0;
  }

  // Writes the map; result gets the path and point count, or the error
  bool save(std::string & result) {
    const std::string format = this->get_parameter("file_format").as_string();
    std::string name = this->get_parameter("map_name").as_string();
    if (format != "pcd" && format != "ply") {
      result = "file_format must be pcd or ply";
    } else if (name.find('/') != std::string::npos) {
      result = "map_name must be a file name, not a path";
    } else {
      const auto pts = grid_->points(this->get_parameter("min_hits").as_int());
      if (pts.empty()) {
        result = "map is empty, nothing saved";
      } else {
        if (name.empty()) {
          char stamp[32];
          const std::time_t now = std::time(nullptr);
          std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&now));
          name = std::string("cloud_map_") + stamp;
        }
        const std::string dir = expand_home(this->get_parameter("output_dir").as_string());
        const std::string path = dir + "/" + name + "." + format;
        if (write_file(dir, path, format, pts, result)) {
          dirty_ = false;
          result = "saved " + std::to_string(pts.size()) + " points (" + target_frame_ +
                   " frame) to " + path;
          RCLCPP_INFO(this->get_logger(), "%s", result.c_str());
          return true;
        }
      }
    }
    RCLCPP_ERROR(this->get_logger(), "Save failed: %s", result.c_str());
    return false;
  }

  // Binary PCD or PLY with float x y z intensity. Written to a .tmp file
  // first, so a failed save never leaves half a map under the real name.
  static bool write_file(const std::string & dir, const std::string & path, const std::string & format,
                         const std::vector<VoxelGrid::Point> & pts, std::string & error) {
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    if (ec) {
      error = "cannot create " + dir + ": " + ec.message();
      return false;
    }
    const std::string tmp = path + ".tmp";
    {
      std::ofstream out(tmp, std::ios::binary);
      if (format == "pcd") {
        out << "# .PCD v0.7 - Point Cloud Data file format\n"
            << "VERSION 0.7\n"
            << "FIELDS x y z intensity\n"
            << "SIZE 4 4 4 4\n"
            << "TYPE F F F F\n"
            << "COUNT 1 1 1 1\n"
            << "WIDTH " << pts.size() << "\n"
            << "HEIGHT 1\n"
            << "VIEWPOINT 0 0 0 1 0 0 0\n"
            << "POINTS " << pts.size() << "\n"
            << "DATA binary\n";
      } else {
        out << "ply\n"
            << "format binary_little_endian 1.0\n"
            << "element vertex " << pts.size() << "\n"
            << "property float x\n"
            << "property float y\n"
            << "property float z\n"
            << "property float intensity\n"
            << "end_header\n";
      }
      out.write(reinterpret_cast<const char *>(pts.data()),
                static_cast<std::streamsize>(pts.size() * sizeof(VoxelGrid::Point)));
      out.close();
      if (!out) {
        std::filesystem::remove(tmp, ec);
        error = "cannot write " + tmp;
        return false;
      }
    }
    std::filesystem::rename(tmp, path, ec);
    if (ec) {
      error = "cannot rename " + tmp + ": " + ec.message();
      return false;
    }
    return true;
  }

  std::string target_frame_;
  double max_range_;
  bool save_on_exit_;

  std::unique_ptr<VoxelGrid> grid_;
  std::unique_ptr<VoxelGrid> preview_;
  bool dirty_ = false;  // points added since the last save

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr preview_pub_;
  rclcpp::TimerBase::SharedPtr preview_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;

  size_t clouds_ = 0;
  size_t dropped_clouds_ = 0;
  size_t points_since_status_ = 0;
};

int main(int argc, char ** argv) {
  // Ctrl-C under ros2 launch arrives twice (from the terminal, then from
  // launch). When rclcpp uninstalls its SIGINT handler it puts back the one
  // it found, so with SIG_DFL there the second Ctrl-C could kill the node
  // before the map was saved. Make that handler SIG_IGN.
  std::signal(SIGINT, SIG_IGN);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CloudMapper>();
  // spin returns on Ctrl-C; the map is still in memory, so save it then
  rclcpp::spin(node);
  node->save_on_exit();
  rclcpp::shutdown();
  return 0;
}
