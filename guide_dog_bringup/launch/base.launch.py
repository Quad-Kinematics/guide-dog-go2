"""Go2 robot base: LiDAR, TF tree and the /cmd_vel bridge.

Shared by robot.launch.py (navigation + app) and mapping.launch.py (SLAM).
Replaces what ~/SLAM/start_navigation.sh started before Nav2 (Hesai driver,
lowstate_to_joint_states, cmd_vel bridge) and the TF part of
go2_navigation.launch.py.

TF: odom -> base_link (odom_to_tf from /utlidar/robot_odom), base_link -> legs
and velodyne (robot_state_publisher, go2_description robot_VLP.xacro, joint
angles from /lowstate), velodyne -> hesai_lidar (identity).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('guide_dog_bringup')
    robot_xacro = os.path.join(
        get_package_share_directory('go2_description'), 'xacro', 'robot_VLP.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Hesai XT16 -> /lidar_points. config/hesai_xt16.yaml is the config the
    # old driver ran with (~/xt16_ws, where that binary was built): it rotates
    # the cloud by yaw 90 deg (transform_flag true), and the maps, AMCL and
    # the costmaps all expect that. ~/SLAM/xt16_ws has an unused copy with no
    # rotation; using it turned every scan 90 deg against the map.
    hesai_node = Node(
        namespace='hesai_ros_driver',
        package='hesai_ros_driver',
        executable='hesai_ros_driver_node',
        output='screen',
        parameters=[{'config_path': os.path.join(bringup_share, 'config', 'hesai_xt16.yaml')}]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro', ' ', robot_xacro]),
            'use_sim_time': use_sim_time
        }]
    )

    # Leg joint angles for robot_state_publisher
    lowstate_to_joint_states_node = Node(
        package='guide_dog_navigation',
        executable='lowstate_to_joint_states',
        name='lowstate_to_joint_states',
        output='screen'
    )

    # odom -> base_link from the Go2's /utlidar/robot_odom
    odom_to_tf_node = Node(
        package='guide_dog_navigation',
        executable='odom_to_tf',
        name='odom_to_tf',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # robot_VLP has base_link -> velodyne; the Hesai cloud is in hesai_lidar
    static_tf_velodyne_to_hesai = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_velodyne_to_hesai',
        arguments=['0', '0', '0', '0', '0', '0', 'velodyne', 'hesai_lidar'],
        output='screen'
    )

    # Legacy frame: some RViz setups use 'base' as the root
    static_tf_base_link_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_link_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base'],
        output='screen'
    )

    # /cmd_vel -> Go2 sport API Move, with accel smoothing (guide_dog_base).
    # Replaces the go2-cmdvel-bridge systemd service: only one may run.
    cmd_vel_bridge_node = Node(
        package='guide_dog_base',
        executable='cmd_vel_bridge',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'),
        hesai_node,
        robot_state_publisher_node,
        lowstate_to_joint_states_node,
        odom_to_tf_node,
        static_tf_velodyne_to_hesai,
        static_tf_base_link_to_base,
        cmd_vel_bridge_node,
    ])
