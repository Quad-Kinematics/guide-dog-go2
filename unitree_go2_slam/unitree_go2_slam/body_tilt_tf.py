"""
Publish base_footprint -> base_link with the trunk's real height, roll and pitch.

The EKF keeps base_footprint flat on the ground (2D mode). A static transform used to put
base_link 0.375 m above it with no tilt, but the trunk really stands at about 0.24 m and pitches
and rolls by several degrees while trotting. The lidar rides on the trunk, so scans cut at a
fixed height above a "level" base_link were full of floor hits whenever the trunk tilted.
With the real tilt in TF, pointcloud_to_laserscan can cut the scan in the level base_footprint
frame instead, where the floor always sits at z = 0.
"""

import math

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


class BodyTiltTF(Node):

    def __init__(self):
        super().__init__('body_tilt_tf')
        # Standing trunk height measured in Gazebo (CHAMP's nominal_height plus the feet).
        self.height = self.declare_parameter('height', 0.241).value
        self.parent = self.declare_parameter('parent_frame', 'base_footprint').value
        self.child = self.declare_parameter('child_frame', 'base_link').value
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Imu, '/imu/data', self._on_imu, qos_profile_sensor_data)

    def _on_imu(self, msg):
        q = msg.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        # Same roll and pitch, no yaw: heading belongs to odom -> base_footprint.
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.parent
        t.child_frame_id = self.child
        t.transform.translation.z = self.height
        t.transform.rotation.w = cr * cp
        t.transform.rotation.x = sr * cp
        t.transform.rotation.y = cr * sp
        t.transform.rotation.z = -sr * sp
        self.broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = BodyTiltTF()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
