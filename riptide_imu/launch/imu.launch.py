import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    this_dir = get_package_share_directory('riptide_imu')

    return LaunchDescription([
        Node(
            package='riptide_imu',
            executable='imu',
            output='screen',
            parameters=[os.path.join(this_dir, 'config', 'vectornav.yaml')]
        )
    ])