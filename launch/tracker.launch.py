from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='creeper_go',
            executable='tracker_node',
            name='tracker_node',
            output='screen',
            parameters=[
                {'server_url': 'http://localhost:8000'}
            ]
        )
    ])