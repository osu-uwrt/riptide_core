from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(name="percentile", default_value="95.0",
                              description="Percentile of each buffer used for the amplitude comparison"),

        Node(
            package='riptide_acoustics',
            executable='run_acoustics',
            name='riptide_acoustics',
            output='screen',
            parameters=[
                {
                    "percentile": LaunchConfiguration("percentile")
                }
            ],
            respawn=True
        )
    ])
