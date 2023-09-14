from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='riptide_imu',
            executable='imu',
            name='imu'
        )
    ])