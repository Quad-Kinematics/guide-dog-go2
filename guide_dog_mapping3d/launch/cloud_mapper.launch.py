"""3D point cloud map from the Go2's built-in L1 LiDAR, for viewing only.

Run it next to robot.launch.py: AMCL gives map -> odom, so the map lines up
with floor_15 and odometry drift is corrected. Without AMCL, pass
target_frame:=odom. Walk the robot around, then save from another shell:
  ros2 service call /cloud_mapper/save std_srvs/srv/Trigger
Ctrl-C also saves. Files go to output_dir (~/maps_3d) as <map_name>.pcd,
or cloud_map_<date>_<time>.pcd when map_name is empty.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('guide_dog_bringup')

    # Typed, so map_name can be empty and a name like 2026 stays a string
    args = [
        ('target_frame', 'map', str, 'Map frame: map (needs AMCL running) or odom'),
        ('voxel_size', '0.05', float, 'Voxel size of the saved map (m)'),
        ('max_range', '15.0', float, 'Drop points farther than this from the robot (m), 0 keeps all'),
        ('output_dir', '~/maps_3d', str, 'Where the map is saved'),
        ('map_name', '', str, 'File name without extension; empty: cloud_map_<date>_<time>'),
        ('file_format', 'pcd', str, 'pcd or ply'),
    ]

    return LaunchDescription([
        # Same DDS setup as robot.launch.py
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable(
            'CYCLONEDDS_URI',
            'file://' + os.path.join(bringup_share, 'config', 'cyclonedds_eth.xml')),

        *[DeclareLaunchArgument(name, default_value=default, description=desc)
          for name, default, _, desc in args],

        Node(
            package='guide_dog_mapping3d',
            executable='cloud_mapper',
            name='cloud_mapper',
            output='screen',
            parameters=[{name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
                         for name, _, value_type, _ in args}],
        ),
    ])
