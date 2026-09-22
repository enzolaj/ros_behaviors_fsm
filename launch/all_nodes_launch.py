"""Launches every node in ros_behaviors_fsm except finite_state_controller:
the safety node and all behavior nodes (drive_square, wall_following,
cookie_follow).

finite_state_controller is left out because it reads terminal input
directly and should be run by itself in its own terminal:
    ros2 run ros_behaviors_fsm finite_state_controller

Usage:
    ros2 launch ros_behaviors_fsm all_nodes_launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'ros_behaviors_fsm'

    return LaunchDescription([
        Node(
            package=package_name,
            executable='safety',
            name='safety',
            output='screen',
        ),
        Node(
            package=package_name,
            executable='drive_square',
            name='drive_square',
            output='screen',
        ),
        Node(
            package=package_name,
            executable='wall_following',
            name='wall_following',
            output='screen',
        ),
        Node(
            package=package_name,
            executable='cookie_follow',
            name='cookie_follow',
            output='screen',
        ),
    ])
