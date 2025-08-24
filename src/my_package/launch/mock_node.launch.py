from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='mock_node',
            name='mock_node',
            output='screen',
            emulate_tty=True,
        )
    ])