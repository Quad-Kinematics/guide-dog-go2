"""Whole guide dog system from this workspace (start_robot.sh runs this).

base.launch.py (LiDAR, TF, cmd_vel bridge) + guide_dog_navigation
navigation.launch.py (AMCL + Nav2) + guide_dog.launch.py (app layer), the app
started app_delay seconds later so Nav2 is up first.
Usage: ros2 launch guide_dog_bringup robot.launch.py [controller:=rpp] [app:=false] [camera:=true]
robot_patrol.launch.py and robot_camera.launch.py are this file with camera off / on.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory('guide_dog_bringup')
    nav_share = get_package_share_directory('guide_dog_navigation')

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'base.launch.py'))
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_share, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'controller': LaunchConfiguration('controller'),
        }.items()
    )

    app_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'guide_dog.launch.py')),
        launch_arguments={
            'viewer_host': LaunchConfiguration('viewer_host'),
            'viewer_port': LaunchConfiguration('viewer_port'),
            'camera': LaunchConfiguration('camera'),
            'perception_images': LaunchConfiguration('perception_images'),
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(nav_share, 'maps', 'floor_15.yaml'),
            description='Map yaml (the patrol waypoints are floor_15 coordinates)'),
        DeclareLaunchArgument(
            'controller',
            default_value=EnvironmentVariable('NAV2_CONTROLLER', default_value='dwb'),
            description='Nav2 controller: dwb or rpp (default: $NAV2_CONTROLLER, else dwb)'),
        DeclareLaunchArgument(
            'viewer_host',
            default_value='0.0.0.0',
            description='Host IP for Yasmin Viewer (an address of this Jetson)'),
        DeclareLaunchArgument(
            'viewer_port',
            default_value='8000',
            description='Port for Yasmin Viewer'),
        DeclareLaunchArgument(
            'app',
            default_value='true',
            description='Start the app layer (FSM, TTS, viewer); false = base + Nav2 only'),
        DeclareLaunchArgument(
            'camera',
            default_value='false',
            description='With the app layer: also start RealSense + perception (person detection)'),
        DeclareLaunchArgument(
            'perception_images',
            default_value='0.0',
            description='Debug: perception saves an annotated frame every this many seconds '
                        '(0 = off) next to its per-frame log in ~/.ros/log/perception_*'),
        DeclareLaunchArgument(
            'app_delay',
            default_value='10.0',
            description='Seconds to wait for Nav2 before starting the app layer'),

        # Every node talks to the Go2 over eth0 through the CycloneDDS 0.10
        # RMW (~/cyclonedds_ws). See config/cyclonedds_eth.xml for why
        # multicast is limited to discovery.
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable(
            'CYCLONEDDS_URI',
            'file://' + os.path.join(bringup_share, 'config', 'cyclonedds_eth.xml')),

        base_launch,
        navigation_launch,
        TimerAction(
            period=LaunchConfiguration('app_delay'),
            actions=[app_launch],
            condition=IfCondition(LaunchConfiguration('app'))
        ),
    ])
