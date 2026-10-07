"""3-point patrol without the camera: robot.launch.py with camera:=false.

The robot walks the patrol waypoints and does the 360° scan at each one, but
nothing publishes /detected_face, so it never stops for a person.
Usage: ros2 launch guide_dog_bringup robot_patrol.launch.py [controller:=rpp]
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
            launch_arguments={'camera': 'false'}.items()
        ),
    ])
