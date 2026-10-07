# Guide Dog Go2

A guide-dog application for the Unitree Go2, built on ROS 2 Foxy. The robot patrols a mapped floor and looks for a person with a RealSense camera and YOLOv8. When it finds one, it turns to face them, greets them through its own speaker and walks them to a fixed destination using Nav2.

Everything runs on the Go2's onboard Jetson (aarch64, Ubuntu 20.04, ROS 2 Foxy).

## Contents

- [How it works](#how-it-works)
- [Repository layout](#repository-layout)
- [Setup](#setup)
- [Running](#running)
- [Building a new map](#building-a-new-map)
- [Configuration](#configuration)
- [Troubleshooting](#troubleshooting)
- [Known limitations](#known-limitations)

## How it works

### Mission state machine

`guide_dog_mission` is a [YASMIN](https://github.com/uleroboticsgroup/yasmin) state machine (C++):

```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> PATROL: START (/start_mission)
    PATROL --> SCAN: waypoint reached
    PATROL --> PATROL: abort (next waypoint)
    PATROL --> ALIGN: person seen (navigation cancelled)
    SCAN --> ALIGN: FACE_DETECTED
    SCAN --> PATROL: NO_FACE
    ALIGN --> GREET: ALIGNED
    ALIGN --> SCAN: LOST_FACE
    GREET --> GUIDE: SPOKEN / FAILED
    GUIDE --> ARRIVE: goal reached
    GUIDE --> IDLE: abort / cancel
    ARRIVE --> IDLE: DONE
```

| State | What it does |
|---|---|
| IDLE | Waits for `/start_mission` |
| PATROL | Sends the next patrol waypoint to Nav2; cancels navigation after 3 person detections in a row |
| SCAN | Turns in place 360° (IMU yaw from `/sportmodestate`) looking for a person with confidence > 0.75 |
| ALIGN | Stands still for 1 s, then turns gently to center the person in the camera image (P-controller on `center_offset_x`) |
| GREET | Speaks a greeting through `/speak` |
| GUIDE | Navigates to the fixed destination |
| ARRIVE | Announces arrival |

The FSM state is shown live in the YASMIN viewer web UI.

### Data flow

```
RealSense ──image──> perception_node (YOLOv8n) ──/detected_face──> main_fsm
Hesai XT16 ──/lidar_points──> scan slices ──/scan, /scan_loc──> AMCL + Nav2
main_fsm ──/navigate_to_pose──> Nav2 ──/cmd_vel──> cmd_vel_bridge ──sport API Move──> Go2
main_fsm ──/speak, /announce──> tts_node ──audio hub API──> Go2 speaker
```

| Interface | Type | Producer → consumer |
|---|---|---|
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | RealSense → perception |
| `/detected_face` | `guide_dog_interfaces/DetectedFace` | perception → PATROL, SCAN, ALIGN |
| `/speak` | `guide_dog_interfaces/srv/Speak` | GREET, ARRIVE → `tts_node` (returns once the phrase has played) |
| `/announce` | `std_msgs/String` | status lines from IDLE, PATROL, SCAN, ALIGN, GUIDE → `tts_node` (fire and forget) |
| `/navigate_to_pose` | `nav2_msgs/NavigateToPose` action | PATROL, GUIDE → Nav2 |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2, SCAN, ALIGN → `cmd_vel_bridge` |
| `/lidar_points` → `/scan`, `/scan_loc` | `PointCloud2` → `LaserScan` | Hesai → costmaps (`/scan`) and AMCL/SLAM (`/scan_loc`) |
| `/utlidar/robot_odom`, `/lowstate`, `/sportmodestate` | Go2 topics | robot → TF, joint states, SCAN |
| `/start_mission` | `std_srvs/Trigger` | operator → IDLE |

### Perception

`perception_node` runs YOLOv8n and publishes the most confident COCO "person" detection (confidence > 0.5) as a `DetectedFace` with `name="Person"` and `center_offset_x` in [-1, 1] (positive means right of center). There is no face recognition yet: the FSM treats any name other than `"Unknown"` as a target.

### Speech

The Jetson has no speaker, so `tts_node` synthesizes each phrase to a WAV, uploads it to the Go2 audio hub over DDS once, and plays it by file ID. Uploaded clips stay on the robot, so only the first use of a new phrase is slow (about 5 to 9 s).

- The default voice is Google TTS (gTTS), which needs internet access. If Google is unreachable, the node falls back to espeak-ng for that phrase. Run with `engine:=espeak` to stay fully offline.
- The Go2 only plays a clip to the end while a WebRTC client is connected. `go2_rtc_keepalive` holds that session open, so **the Unitree mobile app cannot connect while the system is running**.

> **Privacy:** with the default gTTS engine, the text of every new phrase is sent to Google through an unofficial endpoint. Keep personal data out of `/speak` and `/announce`, or use `engine:=espeak`.

## Repository layout

| Package | Language | Contents |
|---|---|---|
| `guide_dog_mission` | C++ | Mission FSM (`main_fsm`) and its states |
| `guide_dog_perception` | Python | YOLOv8 person detector (`perception_node`) |
| `guide_dog_audio` | C++ / Python | `tts_node` (speech on the Go2 speaker), `go2_rtc_keepalive`, venv setup script |
| `guide_dog_interfaces` | msg/srv | `DetectedFace.msg`, `Speak.srv` |
| `guide_dog_base` | C++ | `cmd_vel_bridge`: `/cmd_vel` → Go2 sport API Move, with acceleration smoothing |
| `guide_dog_navigation` | Python / config | AMCL, Nav2 and SLAM launch files, Nav2 params (DWB and RPP), the `floor_15` map, and helper nodes (`odom_to_tf`, `lowstate_to_joint_states`, `goal_pose_relay`, `save_map`, `rviz_click_logger`) |
| `guide_dog_bringup` | launch / scripts | `robot.launch.py`, `base.launch.py`, `mapping.launch.py`, `guide_dog.launch.py`, start scripts, Hesai and CycloneDDS config |

### Third-party packages (not in this repository)

The workspace also needs these packages in `src/`. They are kept out of git (listed in `.git/info/exclude` on the robot):

| Directory | Source | Version on the robot |
|---|---|---|
| `yasmin/` | https://github.com/uleroboticsgroup/yasmin | 6.1.1 (+1 commit, `0b42c472`) |
| `realsense-ros/` | https://github.com/IntelRealSense/realsense-ros | 4.55.1 |
| `HesaiLidar_ROS_2.0/` | https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0 (clone with `--recursive` for the SDK submodule) | v2.0.11 (+6 commits, `96be4a1`) |
| `unitree_msgs/` | `unitree_api` and `unitree_go` from [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) (`cyclonedds_ws/src/unitree`) | must match the robot's SDK |
| `go2_description/` | Go2 URDF (`xacro/robot_VLP.xacro`), copied from `~/go2_desc_ws` on the robot | |

Keep the `unitree_msgs` `.msg` files identical to the robot's SDK. DDS matches messages by their type definitions, so any difference breaks communication with the Go2.

## Setup

### Hardware and network

| Device | Address |
|---|---|
| Go2 | `192.168.123.161` |
| Jetson (`eth0`) | `192.168.123.18` |
| Operator laptop | `192.168.123.100` |

Sensors: Hesai XT16 LiDAR and Intel RealSense camera, both connected to the Jetson. gTTS reaches the internet through the laptop.

### Prerequisites

- ROS 2 Foxy at `/opt/ros/foxy`
- `~/cyclonedds_ws`: CycloneDDS 0.10.2 and `rmw_cyclonedds_cpp`, built as described in the unitree_ros2 README. The Go2 only talks to this RMW version.
- ROS packages:
  ```bash
  sudo apt install ros-foxy-navigation2 ros-foxy-nav2-bringup \
      ros-foxy-nav2-regulated-pure-pursuit-controller ros-foxy-slam-toolbox \
      ros-foxy-pointcloud-to-laserscan ros-foxy-robot-state-publisher ros-foxy-xacro \
      ros-foxy-cv-bridge ros-foxy-tf2-geometry-msgs
  ```
- System libraries: `libespeak-ng1`, `nlohmann-json3-dev`, `libssl-dev`, and librealsense2
- Perception: `ultralytics` for the system Python 3.8 (`/usr/bin/python3 -m pip install ultralytics`; the robot has 8.3.168)
- Audio helpers: a Python 3.10 or newer (the setup script defaults to pyenv's 3.11.9)

### Workspace

```
~/unitree_ros_ws/
├── src/        this repository, plus the third-party packages above
└── venv/       Python venv for gTTS and the WebRTC keepalive (created below)
```

```bash
git clone -b hardware_implementation https://github.com/Quad-Kinematics/guide-dog-go2.git ~/unitree_ros_ws/src
# add the third-party packages to ~/unitree_ros_ws/src (see the table above)
```

### Build

Source the underlay first (the start scripts do this themselves):

```bash
source /opt/ros/foxy/setup.bash
source ~/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Then build:

```bash
cd ~/unitree_ros_ws
colcon build --packages-up-to guide_dog_bringup   # everything the robot runs
src/guide_dog_audio/scripts/setup_venv.sh         # once: creates venv/ (needs internet)
source install/setup.bash
```

Notes:

- Rebuild `guide_dog_interfaces` first after changing `.msg` or `.srv` files, and `unitree_msgs` before anything that uses `unitree_go` or `unitree_api`.
- Launch files, Nav2/SLAM params and maps are installed copies. Rebuild `guide_dog_navigation` or `guide_dog_bringup` after editing them.
- The Jetson is slow. Avoid rebuilding `realsense-ros` and `yasmin` unless you changed them, for example:
  ```bash
  colcon build --packages-select guide_dog_interfaces guide_dog_mission guide_dog_audio guide_dog_perception guide_dog_bringup
  ```
- On the robot, `python3` is pyenv's 3.11, which has no `rclpy`. If `ros2` CLI tools fail, set `PYENV_VERSION=system`.

## Running

### Full system

```bash
ros2 run guide_dog_bringup start_robot.sh
```

This sources the workspaces, refuses to start if SLAM, Nav2 or the Hesai driver is already running, stops the old `go2-cmdvel-bridge` and `go2-odom-to-tf` systemd services if they are active, and runs `robot.launch.py`:

1. `base.launch.py`: Hesai driver, `robot_state_publisher`, joint states, `odom_to_tf`, static TFs, `cmd_vel_bridge`
2. `navigation.launch.py`: LiDAR scan slices, map server, AMCL, Nav2
3. `guide_dog.launch.py` after 10 s: YASMIN viewer, `tts_node`, `go2_rtc_keepalive`, `main_fsm`, and with `camera:=true` the RealSense driver and `perception_node`

Options:

| Option | Default | Effect |
|---|---|---|
| `camera:=true` | `false` | Also start the RealSense driver and person detection. Without it the robot only patrols: it never stops for a person. |
| `NAV2_CONTROLLER=rpp` (env) or `controller:=rpp` | `dwb` | Use Regulated Pure Pursuit instead of DWB |
| `map:=<path to yaml>` | `floor_15.yaml` | Map to localize on. The patrol waypoints are `floor_15` coordinates. |
| `app:=false` | `true` | Start only the base and Nav2 |
| `app_delay:=<seconds>` | `10.0` | Wait before starting the app layer |

Example: `NAV2_CONTROLLER=rpp ros2 run guide_dog_bringup start_robot.sh camera:=true`

### Patrol only, or with the camera

Two launch files fix the `camera` option, for use with `ros2 launch` (source the workspace first; unlike `start_robot.sh`, they don't check for conflicting processes):

| Launch file | What runs |
|---|---|
| `robot_patrol.launch.py` | 3-point patrol: walks the waypoints and does the 360° scan at each one. No camera. |
| `robot_camera.launch.py` | Full mission: patrol plus RealSense and person detection, so the robot stops for a person, turns to them, greets and guides them |

```bash
ros2 launch guide_dog_bringup robot_patrol.launch.py
ros2 launch guide_dog_bringup robot_camera.launch.py controller:=rpp
```

The other `robot.launch.py` options pass through.

### Perception

`perception_node` loads the YOLO weights installed with `guide_dog_perception` (`share/guide_dog_perception/models/yolov8n.pt`; override with the `model_path` parameter), so it runs from any directory. If the file is missing it exits rather than downloading weights. YOLO runs on the Jetson's CPU, at about 5 detections per second with `num_threads` 4 (the default, which leaves the other cores to Nav2).

To run perception on its own, for example with the app layer already up:

```bash
ros2 launch realsense2_camera rs_launch.py initial_reset:=true
ros2 run guide_dog_perception perception_node
```

### Start a mission

AMCL starts without an initial pose (`set_initial_pose: false`), so set one with RViz's "2D Pose Estimate" first. Then:

```bash
ros2 service call /start_mission std_srvs/srv/Trigger
```

### State viewer

Open http://192.168.123.18:8000 from the laptop. The viewer must bind to an address of the Jetson (`viewer_host`), not the laptop's.

### App layer only

If the base and Nav2 are already running:

```bash
ros2 launch guide_dog_bringup guide_dog.launch.py viewer_host:=0.0.0.0 viewer_port:=8000
```

## Building a new map

```bash
ros2 run guide_dog_bringup start_mapping.sh
```

This starts the base and SLAM Toolbox on `/scan_loc`, the same LiDAR slice AMCL uses. Walk the robot through the floor with the remote, then save the map from another shell:

```bash
cd ~/unitree_ros_ws/src/guide_dog_navigation/maps
ros2 run guide_dog_navigation save_map <name>
```

Rebuild `guide_dog_navigation` to install the new map, or pass `map:=<path to yaml>` to the launch. The patrol waypoints and the guide destination must then be updated for the new map (see below).

## Configuration

| What | Where |
|---|---|
| Patrol waypoints | `guide_dog_mission/src/states/patrol_state.cpp` (`map` frame, `floor_15` coordinates) |
| Guide destination | `guide_dog_mission/src/states/guide_state.cpp` |
| Nav2 parameters | `guide_dog_navigation/config/nav2_params_dwb.yaml`, `nav2_params_rpp.yaml`. Keep them in sync outside `controller_server.FollowPath` and the local inflation layer. |
| Person-detection debounce | `min_detections_`, `max_detection_gap_sec_` in `guide_dog_mission/include/guide_dog_mission/states/patrol_state.hpp` |
| ALIGN gain and balance limits (settle time, yaw rate range, timeouts) | `guide_dog_mission/include/guide_dog_mission/states/align_state.hpp` |
| YOLO model and CPU threads | `perception_node` parameters `model_path`, `num_threads` |
| Velocity limits and smoothing | `cmd_vel_bridge` parameters (`max_vx`, `max_vy`, `max_wz`, `accel_*`, `decel_*`, `smooth`) in `guide_dog_base/src/cmd_vel_bridge.cpp` |
| Speech engine and voice | `tts_node` parameters (`engine`, `gtts_lang`, `voice`, `rate_wpm`, `rtc_wait_sec`) |
| LiDAR mounting | `guide_dog_bringup/config/hesai_xt16.yaml` |
| DDS network setup | `guide_dog_bringup/config/cyclonedds_eth.xml` |

To add a mission state, put the header in `guide_dog_mission/include/guide_dog_mission/states/`, the source in `src/states/`, add it to `add_executable` in `CMakeLists.txt`, and wire it up in `setup_state_machine()` in `main_fsm.cpp`.

## Troubleshooting

| Symptom | Cause and fix |
|---|---|
| Phantom obstacles, AMCL jumps, DWB reports "Resulting plan has 0 poses" | The LiDAR cloud is not rotated. The Hesai driver must run with `guide_dog_bringup/config/hesai_xt16.yaml`, which rotates it by yaw 90°. The map, AMCL and costmaps all expect that rotation. |
| Two `/cmd_vel` bridges running (for example the old `go2-cmdvel-bridge` service) | Both would drive the robot. Only the one in `base.launch.py` should run; keep the systemd service disabled. |
| Nothing moves the robot outside the launch files | Expected: `cmd_vel_bridge` only runs inside `robot.launch.py` and `mapping.launch.py`. |
| Speech stops after a fraction of a second | No WebRTC session is open. Check that `go2_rtc_keepalive` is running and that the Unitree app is not connected. |
| First use of a phrase is slow | Expected: the phrase is generated and uploaded once (about 5 to 9 s), then cached on the robot. |
| Nav2 aborts with "transform too old" | DDS multicast lag on `eth0`. Use the shipped `cyclonedds_eth.xml`, which limits multicast to discovery (`AllowMulticast=spdp`). |
| `ros2` CLI errors about `rclpy` | pyenv's Python is active. Set `PYENV_VERSION=system`. |

## Known limitations

- No face recognition: any detected person is treated as a target.
- Patrol waypoints and the guide destination are hardcoded for `floor_15`, and their orientation (yaw) is not set.
- There are no functional tests. The Python packages only have the standard ament lint tests:
  ```bash
  colcon test --packages-select guide_dog_perception && colcon test-result --verbose
  ```
- Most packages do not declare a license yet (`TODO` in `package.xml`). `guide_dog_navigation` is MIT. The third-party packages keep their own licenses.
