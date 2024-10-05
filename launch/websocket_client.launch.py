import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('yaml_file', default_value='topics.yaml', description='Path to the YAML file'),
        DeclareLaunchArgument('ws_url', default_value='ws://localhost:9090', description='WebSocket server URL'),

        Node(
            package='ros2_websocket_proxy',
            executable='generic_subscriber_client',
            name='generic_subscriber_client',
            output='screen',
            parameters=[{
                'yaml_file': LaunchConfiguration('yaml_file'),
                'ws_url': LaunchConfiguration('ws_url'),
            }],
        ),
    ])
