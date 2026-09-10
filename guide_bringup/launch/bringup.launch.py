#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the combined launch description for the packages we have today."""
    # One on/off switch per package. Default is "on" so a plain
    # `ros2 launch guide_bringup bringup.launch.py` starts everything that
    # exists, matching how face_recognition.launch.py behaves on its own.
    enable_perception_arg = DeclareLaunchArgument(
        'enable_perception',
        default_value='true',
        description='Start guide_perception_extended (RealSense camera + face_recognition_node).',
    )
    enable_cognition_arg = DeclareLaunchArgument(
        'enable_cognition',
        default_value='true',
        description='Start guide_cognition (greeting_node).',
    )

    enable_perception = LaunchConfiguration('enable_perception')
    enable_cognition = LaunchConfiguration('enable_cognition')

    # guide_perception_extended already has its own standalone launch file that stands
    # up the RealSense driver and face_recognition_node together. Reuse it
    # here instead of duplicating that camera configuration a second time.
    perception_launch_path = os.path.join(
        get_package_share_directory('guide_perception_extended'),
        'launch',
        'face_recognition.launch.py',
    )
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(perception_launch_path),
        condition=IfCondition(enable_perception),
    )

    # guide_cognition does not have its own launch file yet, so greeting_node
    # is started directly here with its package's own parameters file.
    cognition_params_file = os.path.join(
        get_package_share_directory('guide_cognition'),
        'config',
        'guide_cognition_params.yaml',
    )
    cognition_node = Node(
        package='guide_cognition',
        executable='greeting_node',
        name='greeting_node',
        output='screen',
        parameters=[cognition_params_file],
        condition=IfCondition(enable_cognition),
    )

    return LaunchDescription([
        enable_perception_arg,
        enable_cognition_arg,
        perception_launch,
        cognition_node,
    ])
