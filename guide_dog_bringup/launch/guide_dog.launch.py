from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Declare Launch Arguments (Allows overriding via command line)
    viewer_host_arg = DeclareLaunchArgument(
        'viewer_host',
        default_value='0.0.0.0',
        description='Host IP for Yasmin Viewer'
    )
    
    viewer_port_arg = DeclareLaunchArgument(
        'viewer_port',
        default_value='8000',
        description='Port for Yasmin Viewer'
    )

    camera_arg = DeclareLaunchArgument(
        'camera',
        default_value='false',
        description='Start the RealSense driver and the perception node'
    )

    return LaunchDescription([
        viewer_host_arg,
        viewer_port_arg,
        camera_arg,

        # 2. Start Yasmin Viewer with custom host and port
        Node(
            package='yasmin_viewer',
            executable='yasmin_viewer_node',
            name='yasmin_viewer',
            parameters=[{
                'host': LaunchConfiguration('viewer_host'),
                'port': LaunchConfiguration('viewer_port')
            }]
        ),

        # 3. Start Realsense Camera (camera:=true). FindPackageShare is only
        # resolved when the condition holds, so camera:=false runs without it.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])),
            launch_arguments={'initial_reset': 'true'}.items(),
            condition=IfCondition(LaunchConfiguration('camera'))
        ),

        # 4. Start your custom perception (camera:=true): YOLOv8 person
        # detector, publishes /detected_face
        Node(
            package='guide_dog_perception',
            executable='perception_node',
            name='guide_dog_perception',
            output='screen',
            condition=IfCondition(LaunchConfiguration('camera'))
        ),

        # 5. Start audio processing
        Node(
            package='guide_dog_audio',
            executable='tts_node',
            name='guide_dog_audio'
        ),

        # Keeps a WebRTC session open; without one the Go2 speaker stops each
        # clip after a fraction of a second (not a ROS node)
        Node(
            package='guide_dog_audio',
            executable='go2_rtc_keepalive',
            name='go2_rtc_keepalive',
            output='screen'
        ),

        # 6. Start the Yasmin state machine / mission control
        Node(
            package='guide_dog_mission',
            executable='main_fsm', 
            name='guide_dog_mission'
        )
    ])
