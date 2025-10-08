#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    acceleration_arg = DeclareLaunchArgument(
        'acceleration',
        default_value='10000',
        description='Motor acceleration parameter'
    )
    
    deceleration_arg = DeclareLaunchArgument(
        'deceleration',
        default_value='20000',
        description='Motor deceleration parameter'
    )
    
    reset_motors_flag_arg = DeclareLaunchArgument(
        'reset_motors_flag',
        default_value='false',
        description='Reset motors flag'
    )

    # KaCanOpen espeleo bridge node
    kacanopen_espeleo_node = Node(
        package='kacanopen',
        executable='kacanopen_espeleo_bridge',
        name='kacanopen_espeleo',
        output='screen',
        respawn=True,
        parameters=[{
            'acceleration': LaunchConfiguration('acceleration'),
            'deceleration': LaunchConfiguration('deceleration'),
            'reset_motors_flag': LaunchConfiguration('reset_motors_flag'),
        }]
    )

    return LaunchDescription([
        acceleration_arg,
        deceleration_arg,
        reset_motors_flag_arg,
        kacanopen_espeleo_node,
    ])
