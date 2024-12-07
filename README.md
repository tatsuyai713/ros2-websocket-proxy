# ROS 2 WebSocket Proxy

## Overview

The `ros2_websocket_proxy` package provides a bridge between ROS 2 topics and WebSocket connections. It allows ROS 2 nodes to publish and subscribe to topics over WebSocket, enabling communication with web clients or other ROS 2 networks. This package includes two main components: a generic subscriber client and a generic publisher server.

## Features

- **Generic Subscription**: The subscriber client can listen to any ROS 2 topic specified in a YAML configuration file.
- **Generic Publishing**: The publisher server can publish messages to any ROS 2 topic, also specified in a YAML configuration file.
- **WebSocket Support**: Facilitates communication over WebSocket, making it suitable for web applications or real-time data streaming.
- **Dynamic Configuration**: Users can specify topics and types in a YAML file, allowing for flexible configuration without modifying code.

## Installation

To install the `ros2_websocket_proxy` package, clone the repository and build it within your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/tatsuyai713/ros2-websocket-proxy
cd ~/ros2_ws
colcon build
```

## Configuration

The package uses a YAML file to specify the topics for subscription and publication. The YAML file should be placed in the `config` folder of the package. The file format should look like this:

```yaml
publish_topics:
  - name: "/test"
    type: "std_msgs/msg/Float64"
subscribe_topics:
  - name: "/another_topic"
    type: "sensor_msgs/msg/Image"

```

## Usage

### Starting the Generic Subscriber Client

To run the generic subscriber client, use the following command:

```bash
ros2 run ros2_websocket_proxy generic_client --ros-args -p yaml_file:=<path_to_yaml_file> -p ws_url:=<websocket_url>
```

### Starting the Generic Publisher Server

To run the generic publisher server, use the following command:

```bash
ros2 run ros2_websocket_proxy generic_server --ros-args -p yaml_file:=<path_to_yaml_file>
```

### Launch File

You can also use a launch file to start the nodes with specified parameters. Here's an example launch file:

```python
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('yaml_file', default_value='client_topics.yaml', description='Path to the YAML file'),
        DeclareLaunchArgument('ws_url', default_value='ws://localhost:9090', description='WebSocket server URL'),

        Node(
            package='ros2_websocket_proxy',
            executable='generic_client',
            name='generic_client',
            output='screen',
            parameters=[{
                'yaml_file': LaunchConfiguration('yaml_file'),
                'ws_url': LaunchConfiguration('ws_url'),
            }],
        ),
    ])
```

```python
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
```
