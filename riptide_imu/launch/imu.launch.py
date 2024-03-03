import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    this_dir = get_package_share_directory('riptide_imu')

    return LaunchDescription([
        DeclareLaunchArgument(name="port", default_value="/dev/ttyTHS0",
                              description="Port to connect over"),

        Node(
            package='riptide_imu',
            executable='imu',
            output='screen',
            parameters=[
                os.path.join(this_dir, 'config', 'vectornav.yaml'),
                {
                    "port" : LaunchConfiguration("port"),
                }
            ]
        )
    ])