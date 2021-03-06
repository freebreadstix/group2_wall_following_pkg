from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='group2_wall_following_pkg',
            executable='wall_follow',
            output='screen'),
    ])
