#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    acceleration_arg = DeclareLaunchArgument(
        'acceleration',
        default_value='100000',
        description='Motor acceleration parameter'
    )

    # Callback test problem node
    callback_test_node = Node(
        package='kacanopen',
        executable='kacanopen_callback_test_problem',
        name='callback_test_problem',
        output='screen',
        parameters=[{
            'acceleration': LaunchConfiguration('acceleration'),
        }]
    )

    return LaunchDescription([
        acceleration_arg,
        callback_test_node,
    ])
