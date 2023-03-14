from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mpu9250',
            executable='mpu9250_node',
            name='mpu9250',
        ),
    ])
