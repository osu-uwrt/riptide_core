from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(name="mode", default_value="max",
                              description="Mode of comparing buffer 0 and buffer 1 to find nearest pinger"),

        Node(
            package='riptide_acoustics',
            executable='run_acoustics',
            name='riptide_acoustics',
            output='screen',
            parameters=[
                {
                    "mode": LaunchConfiguration("mode")
                }
            ],
            respawn=True
        )
    ])
