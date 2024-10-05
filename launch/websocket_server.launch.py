import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('yaml_file', default_value='topics.yaml', description='Path to the YAML file'),

        Node(
            package='ros2_websocket_proxy',
            executable='generic_publisher_server',
            name='generic_publisher_server',
            output='screen',
            parameters=[{
                'yaml_file': LaunchConfiguration('yaml_file'),
            }],
        ),
    ])
