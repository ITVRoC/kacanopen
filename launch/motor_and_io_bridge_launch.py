from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kacanopen',
            executable='kacanopen_ros2_motor_and_io_bridge',
            name='motor_and_io_bridge',
            output='screen',
            parameters=[
                {'busname': 'can0'},
                {'baudrate': '1M'},
                {'acceleration': 10000},
                {'deceleration': 20000},
                {'heartbeat_interval': 100},
                {'publish_rate': 50.0},
            ],
            remappings=[
                # You can remap topics here if needed
                # ('joint_states', 'robot/joint_states'),
            ]
        )
    ])


