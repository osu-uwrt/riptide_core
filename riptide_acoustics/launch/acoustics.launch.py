#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_ns = LaunchConfiguration('robot', default='talos')
    return LaunchDescription([
        PushRosNamespace(robot_ns),
        Node(
            package='riptide_acoustics',
            executable='localization',
            name='localization',
            output='screen',
            parameters=[
                {'sound_speed_formula': 'UNESCO'},
                {'default_salinity': 0.0}
            ]
        )
    ])