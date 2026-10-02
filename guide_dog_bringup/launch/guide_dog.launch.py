import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

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

    return LaunchDescription([
        viewer_host_arg,
        viewer_port_arg,

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

        # 3. Start Realsense Camera
        # 1. Start Realsense Camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')]),
	    launch_arguments={'initial_reset': 'true'}.items(),
        ),
        
        # 4. Start your custom perception
        #Node(
        #    package='guide_dog_perception',
        #    executable='perception_node', 
        #    name='guide_dog_perception'
        #),

        # 5. Start audio processing
        Node(
            package='guide_dog_audio',
            executable='tts_node', 
            name='guide_dog_audio'
        ),

        # 6. Start the Yasmin state machine / mission control
        Node(
            package='guide_dog_mission',
            executable='main_fsm', 
            name='guide_dog_mission'
        )
    ])
