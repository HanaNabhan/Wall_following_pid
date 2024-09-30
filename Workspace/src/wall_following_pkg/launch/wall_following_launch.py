# ~/ros2_workspace/src/wall_following_pkg/launch/wall_following_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wall_following_pkg',
            executable='lidar_processing_node',
            name='lidar_processing_node',
            output='screen'
        ),
        Node(
            package='wall_following_pkg',
            executable='control_node',
            name='control_node',
            output='screen'
        )
    ])
