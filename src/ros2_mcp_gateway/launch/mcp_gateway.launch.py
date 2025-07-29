from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ros2_mcp_gateway'),
        'config',
        'mcp_gateway.yaml'
    )

    return LaunchDescription([
        Node(
            package='ros2_mcp_gateway',
            executable='gateway_node',
            name='mcp_gateway',
            output='screen',
            parameters=[config]
        )
    ])