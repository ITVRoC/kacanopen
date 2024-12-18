
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kacanopen',
            executable='/mnt/data/kacanopen_converted/launch/test_callback',  # Example executable
            name='test_callback',
            parameters=[
                {"busname": "slcan0"},  # Add your parameters here
                {"baudrate": "500K"}
            ],
            output='screen'
        )
    ])
