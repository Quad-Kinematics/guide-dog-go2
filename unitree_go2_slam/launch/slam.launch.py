import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    # --- PACKAGE PATHS ---
    go2_slam_pkg = get_package_share_directory("unitree_go2_slam")
    go2_desc_pkg = get_package_share_directory("unitree_go2_description")

    # --- CONFIG PATHS ---
    joints_config = os.path.join(go2_slam_pkg, "config/joints/joints.yaml")
    ros_control_config = os.path.join(
        go2_slam_pkg, "config/ros_control/ros_control.yaml")
    gait_config = os.path.join(go2_slam_pkg, "config/gait/gait.yaml")
    links_config = os.path.join(go2_slam_pkg, "config/links/links.yaml")
    urdf_path = os.path.join(go2_desc_pkg, "urdf/unitree_go2_robot.xacro")
    rviz_config_path = os.path.join(go2_slam_pkg, "rviz/rviz.rviz")
    world = LaunchConfiguration("world")
    map_yaml = LaunchConfiguration("map")
    nav2_params = LaunchConfiguration("params_file")
    headless = LaunchConfiguration("headless")

    # --- 1. ROBOT STATE PUBLISHER ---
    robot_description = {"robot_description": Command(
        ["xacro ", urdf_path, " robot_controllers:=", ros_control_config])}
    robot_state_publisher_node = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        output="screen", parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # --- 2. GAZEBO SIMULATION & SPAWN ---
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    # Headless runs only the Gazebo server; the lidar and camera still render off-screen.
    gz_headless_args = PythonExpression(
        ["' -s --headless-rendering' if '", headless, "' == 'true' else ''"])
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [world, ' -r', gz_headless_args]}.items(),
    )

    gazebo_spawn_robot = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', 'go2', '-topic', 'robot_description',
                   '-x', '0.0', '-y', '0.0', '-z', '0.375'],
    )

    # --- Pull Ground Truth Odometry ---
    gazebo_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/velodyne_points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/joint_group_effort_controller/joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory',
            # '/rgb_image@sensor_msgs/msg/Image@gz.msgs.Image',
            # D455 RGBD camera bridges
            '/d455/image@sensor_msgs/msg/Image[gz.msgs.Image',
            # '/d455/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            # '/d455/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # '/d455/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ]
    )

    # --- 3. QUADRUPED CONTROLLERS ---
    quadruped_controller_node = Node(
        package="champ_base", executable="quadruped_controller_node", output="screen",
        parameters=[
            {"use_sim_time": use_sim_time}, {"gazebo": True}, {
                "publish_joint_states": True},
            {"publish_joint_control": True}, {"publish_foot_contacts": False},
            {"joint_controller_topic": "joint_group_effort_controller/joint_trajectory"},
            {"urdf": Command(['xacro ', urdf_path])
             }, joints_config, links_config, gait_config,
            {"hardware_connected": False}, {"close_loop_odom": False},
        ],
        remappings=[("/cmd_vel/smooth", "/cmd_vel")],
    )

    # --- 4. TF TREE ---
    footprint_to_odom_ekf = Node(
        package="robot_localization", executable="ekf_node", name="footprint_to_odom_ekf",
        parameters=[
            {"use_sim_time": use_sim_time}, {
                "base_link_frame": "base_footprint"},
            {"odom_frame": "odom"}, {"world_frame": "odom"}, {
                "publish_tf": True}, {"frequency": 50.0},
            {"two_d_mode": True}, {"odom0": "odom"},
            # x, y and yaw from Gazebo's ground truth, plus vx, vy and yaw rate. Without absolute
            # yaw its variance grows without bound, and because /odom has zero covariance a 1 cm
            # gait sway in x/y could then flip the heading by 100+ degrees in one update.
            {"odom0_config": [True, True, False, False, False, True,
                              True, True, False, False, False, True, False, False, False]}
        ]
    )

    # The trunk's real height and IMU tilt; a static, level transform made the lidar sweep the
    # floor into the scan whenever the trunk pitched.
    base_footprint_to_base_link_tf = Node(
        package='unitree_go2_slam', executable='body_tilt_tf', name='body_tilt_tf',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    controller_spawner_js = TimerAction(period=20.0, actions=[Node(
        package="controller_manager", executable="spawner", arguments=["joint_states_controller"])])
    controller_spawner_effort = TimerAction(period=25.0, actions=[Node(
        package="controller_manager", executable="spawner", arguments=["joint_group_effort_controller"])])

    # --- 5. RVIZ ---
    rviz_node = Node(package='rviz2', executable='rviz2', arguments=[
                     '-d', rviz_config_path], parameters=[{'use_sim_time': use_sim_time}],
                     condition=UnlessCondition(headless))

    # --- 6. 2D LASER CONVERTER ---
    pc_to_laserscan = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', '/velodyne_points/points'),
                    ('scan', '/scan')],
        # Cut the scan in the level base_footprint frame: heights are above the floor, so floor
        # hits (z ~ 0) stay out however the trunk tilts. range_min drops the robot's own legs
        # (the footprint reaches 0.40 m from the centre); nothing closer is visible to the lidar.
        parameters=[{'target_frame': 'base_footprint', 'min_height': 0.15, 'max_height': 1.5,
                     'angle_increment': 0.0087,
                     'scan_time': 0.1, 'range_min': 0.45, 'range_max': 30.0, 'use_inf': True, 'use_sim_time': True}],
    )

    # --- 7. NAV2 BRINGUP ---
    nav2_launch = TimerAction(
        period=30.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py'])),
            launch_arguments={'use_sim_time': 'true',
                              'map': map_yaml,
                              'params_file': nav2_params}.items()
        )]
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "world", default_value=os.path.join(go2_slam_pkg, "worlds/room_human_aligned.world"),
            description="Gazebo world; worlds/room.world has a walking person instead"),
        DeclareLaunchArgument(
            "map", default_value=os.path.join(go2_slam_pkg, "maps/arena_map.yaml"),
            description="Map for AMCL and the global costmap"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(go2_slam_pkg, "config/nav2/nav2_params.yaml"),
            description="Nav2 parameters, tuned for the Go2"),
        DeclareLaunchArgument(
            "headless", default_value="false",
            description="Run Gazebo without its GUI and skip RViz"),
        robot_state_publisher_node, gz_sim, gazebo_spawn_robot, gazebo_bridge,
        quadruped_controller_node, footprint_to_odom_ekf, base_footprint_to_base_link_tf,
        controller_spawner_js, controller_spawner_effort,
        rviz_node, pc_to_laserscan, nav2_launch
    ])
