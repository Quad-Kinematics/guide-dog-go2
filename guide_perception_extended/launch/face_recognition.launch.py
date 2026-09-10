#!/usr/bin/env python3
"""Standalone bring-up for the guide_perception_extended identity stack.

WHAT THIS IS
------------
Launches guide_perception_extended on its own: the native Intel RealSense camera driver
plus the face_recognition_node that consumes its colour stream and publishes
identity results on /recognized_person.

WHY IT EXISTS (the important part)
----------------------------------
guide_perception_extended must be runnable WITHOUT Stack B's master bringup
(robot_state_bringup.launch.py). An earlier session had wired the
face_recognition_node into that master launch as a shortcut to reuse its
RealSense bring-up; that coupled a decoupled-by-design identity process to the
Legs/Brain/Mouth/Nav stack. This launch file undoes that: it stands the camera
up itself and reads guide_perception_extended's OWN config, so nothing here depends on
wso2_unitree_bringup or any other Stack B package.

HOW TO RUN
----------
    ros2 launch guide_perception_extended face_recognition.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description: RealSense camera + face_recognition_node."""
    # guide_perception_extended's own parameters (no Stack B config dependency).
    params_file = os.path.join(
        get_package_share_directory('guide_perception_extended'),
        'config',
        'face_recognition_params.yaml',
    )

    # Native Intel RealSense driver. Namespaced under `camera` so colour frames
    # publish on /camera/color/image_raw (what face_recognition_node subscribes
    # to). These settings match the camera bring-up guide_perception_extended was
    # previously borrowing from Stack B's master launch.
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense_camera',
        namespace='camera',
        output='screen',
        parameters=[{
            'enable_depth': True,
            'enable_color': True,
            'enable_infra1': False,
            'enable_infra2': False,
            'depth_module.profile': '640x480x30',
            'rgb_module.profile': '640x480x30',
            'pointcloud.enable': True,
            'pointcloud.ordered_pc': False,
            'align_depth.enable': True,
        }],
    )

    # Identity node. Its own process, own executor — a slow/crashed InsightFace
    # inference can never stall any other node.
    face_recognition_node = Node(
        package='guide_perception_extended',
        executable='face_recognition_node',
        name='face_recognition_node',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        realsense_node,
        face_recognition_node,
    ])
