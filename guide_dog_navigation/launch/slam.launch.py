"""SLAM Toolbox mapping for the Go2 (replaces ~/SLAM/start_slam.sh).

Expects the robot base to be running (guide_dog_bringup base.launch.py).
Maps are built from the same scan slice AMCL localizes with
(pointcloud_to_laserscan_loc.yaml, /scan_loc), so a new map works with
navigation.launch.py as is. Save with: ros2 run guide_dog_navigation save_map <name>
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('guide_dog_navigation')

    pointcloud_to_laserscan_loc_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_loc',
        output='screen',
        remappings=[
            ('cloud_in', LaunchConfiguration('cloud_topic')),
            ('scan', '/scan_loc')
        ],
        parameters=[os.path.join(share, 'config', 'pointcloud_to_laserscan_loc.yaml')]
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[LaunchConfiguration('slam_params_file')],
        remappings=[
            ('scan', '/scan_loc'),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(share, 'config', 'slam_params.yaml'),
            description='SLAM Toolbox params file'),
        DeclareLaunchArgument(
            'cloud_topic',
            default_value='/lidar_points',
            description='Input PointCloud2 topic for pointcloud_to_laserscan'),
        pointcloud_to_laserscan_loc_node,
        slam_toolbox_node,
    ])
