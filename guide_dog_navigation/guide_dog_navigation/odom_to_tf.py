import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

QOS_RELIABLE = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=50,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)

class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.br = TransformBroadcaster(self)
        self.count = 0
        self.last_odom_stamp = None
        # Upper bound only: TF is published from the odom callback (~150 Hz input)
        self.publish_rate_hz = float(
            self.declare_parameter('publish_rate_hz', 50.0).value
        )
        if self.publish_rate_hz <= 0.0:
            self.publish_rate_hz = 50.0
        self.min_publish_period_ns = int(1e9 / self.publish_rate_hz)
        self.last_publish_ns = None
        self.log_timer = self.create_timer(1.0, self._tick)

        self.sub = self.create_subscription(
            Odometry,
            '/utlidar/robot_odom',
            self.cb,
            QOS_RELIABLE
        )

        self.get_logger().info("odom_to_tf started (RELIABLE sub)")

    def _tick(self):
        if self.last_odom_stamp is None:
            self.get_logger().info("waiting for odom messages...")
        else:
            now = self.get_clock().now().to_msg()
            self.get_logger().info(
                f"rx odom msgs: {self.count}, "
                f"odom stamp: {self.last_odom_stamp.sec}.{self.last_odom_stamp.nanosec:09d}, "
                f"tf stamp(now): {now.sec}.{now.nanosec:09d}"
            )

    def cb(self, msg: Odometry):
        self.count += 1
        self.last_odom_stamp = msg.header.stamp

        # Publish here rather than from a timer. The timer re-stamped the latest
        # odom with now(), so the pose could be up to a full period older than its
        # stamp: ~3 deg of heading error at 0.9 rad/s, which AMCL corrected during
        # turns and then snapped back from. It also kept publishing stale poses
        # as fresh if odom stopped.
        now = self.get_clock().now()
        if (self.last_publish_ns is not None
                and now.nanoseconds - self.last_publish_ns < self.min_publish_period_ns):
            return
        self.last_publish_ns = now.nanoseconds

        t = TransformStamped()
        # Stamp with host time: the Go2 clock is not synced to the Jetson's
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = OdomToTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
