"""Build a new map: robot base + SLAM Toolbox (start_mapping.sh runs this).

Drive the robot around with the remote, then save from another shell:
  ros2 run guide_dog_navigation save_map <name>
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_share = get_package_share_directory('guide_dog_bringup')
    nav_share = get_package_share_directory('guide_dog_navigation')

    return LaunchDescription([
        # Same DDS setup as robot.launch.py
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable(
            'CYCLONEDDS_URI',
            'file://' + os.path.join(bringup_share, 'config', 'cyclonedds_eth.xml')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'base.launch.py'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_share, 'launch', 'slam.launch.py'))
        ),
    ])
