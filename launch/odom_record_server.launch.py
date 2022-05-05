from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sim_pkg',
            executable='recordOdom_server',
            output='screen'),
    ])
