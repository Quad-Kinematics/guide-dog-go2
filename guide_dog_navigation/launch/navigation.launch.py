"""Localization (map_server + AMCL) and Nav2 for the Go2.

Expects the robot base to be running (guide_dog_bringup base.launch.py): Hesai
driver on /lidar_points, odom -> base_link TF, robot_state_publisher.
Usage: ros2 launch guide_dog_navigation navigation.launch.py [controller:=rpp] [map:=<yaml>]
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def launch_setup(context):
    share = get_package_share_directory('guide_dog_navigation')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    cloud_topic = LaunchConfiguration('cloud_topic')

    # controller:=dwb|rpp picks config/nav2_params_<controller>.yaml unless
    # params_file is given
    params_file = LaunchConfiguration('params_file').perform(context)
    if not params_file:
        controller = LaunchConfiguration('controller').perform(context)
        params_file = os.path.join(share, 'config', f'nav2_params_{controller}.yaml')
    if not os.path.isfile(params_file):
        raise RuntimeError(
            f'Nav2 params file not found: {params_file} (controller must be dwb or rpp)')

    # Obstacle slice for the costmaps
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', cloud_topic),
            ('scan', '/scan')
        ],
        parameters=[os.path.join(share, 'config', 'pointcloud_to_laserscan.yaml')]
    )

    # Second scan for AMCL only, sliced the way the map was built
    # (see pointcloud_to_laserscan_loc.yaml). The costmaps keep using /scan.
    pointcloud_to_laserscan_loc_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_loc',
        output='screen',
        remappings=[
            ('cloud_in', cloud_topic),
            ('scan', '/scan_loc')
        ],
        parameters=[os.path.join(share, 'config', 'pointcloud_to_laserscan_loc.yaml')]
    )

    # Relay PoseStamped /goal_pose messages into NavigateToPose action
    goal_pose_relay_node = Node(
        package='guide_dog_navigation',
        executable='goal_pose_relay',
        name='goal_pose_relay',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_goal_pose_relay')),
        parameters=[{'use_sim_time': use_sim_time}]
    )

    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {
                'yaml_filename': LaunchConfiguration('map'),
                'use_sim_time': use_sim_time
            }
        ]
    )

    amcl_node = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'global_frame_id': 'map',
                'scan_topic': LaunchConfiguration('scan_topic'),
                'tf_broadcast': True
            }
        ]
    )

    localization_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server', 'amcl']
        }]
    )

    # Nav2 (controller, planner, recoveries, bt_navigator, waypoint_follower)
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'map_subscribe_transient_local': 'true'
        }.items()
    )

    return [
        pointcloud_to_laserscan_node,
        pointcloud_to_laserscan_loc_node,
        goal_pose_relay_node,
        map_server_node,
        amcl_node,
        localization_lifecycle_manager,
        nav2_bringup_launch,
    ]


def generate_launch_description():
    share = get_package_share_directory('guide_dog_navigation')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(share, 'maps', 'floor_15.yaml'),
            description='Full path to map yaml file'),
        DeclareLaunchArgument(
            'controller',
            default_value='dwb',
            description='Nav2 controller: dwb (DWB) or rpp (Regulated Pure Pursuit)'),
        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description='Nav2 params file; overrides controller when set'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan_loc',
            description='Scan topic for AMCL: /scan_loc matches the slice floor_15 was '
                        'built from; /scan is the obstacle slice the costmaps use'),
        DeclareLaunchArgument(
            'cloud_topic',
            default_value='/lidar_points',
            description='Input PointCloud2 topic for pointcloud_to_laserscan'),
        DeclareLaunchArgument(
            'enable_goal_pose_relay',
            default_value='true',
            description='Relay /goal_pose (e.g. RViz over rosbridge) to NavigateToPose'),
        OpaqueFunction(function=launch_setup),
    ])
