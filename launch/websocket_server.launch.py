import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('yaml_file', default_value='server_topics.yaml', description='Path to the YAML file'),
        DeclareLaunchArgument('port', default_value='9090', description='WebSocket server port'),
        Node(
            package='ros2_websocket_proxy',
            executable='generic_server',
            name='generic_server',
            output='screen',
            parameters=[{
                'yaml_file': LaunchConfiguration('yaml_file'),
                'port': LaunchConfiguration('port'),
            }],
        ),
    ])
