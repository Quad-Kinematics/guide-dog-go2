"""Full mission with person detection: robot.launch.py with camera:=true.

Adds the RealSense driver and perception_node (YOLOv8, /detected_face) to the
app layer, so PATROL and SCAN stop for a person, ALIGN turns to them, and the
robot greets and guides them.
Usage: ros2 launch guide_dog_bringup robot_camera.launch.py [controller:=rpp]
Other robot.launch.py arguments (map, controller, viewer_host, viewer_port,
app_delay) pass through.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_share = get_package_share_directory('guide_dog_bringup')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'robot.launch.py')),
            launch_arguments={'camera': 'true'}.items()
        ),
    ])
