import os
from ament_index_python import get_package_share_directory
from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration as LC

default_config_file = os.path.join(
    get_package_share_directory("riptide_gyro"),
    "config",
    "gyro_config.yaml"
)

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value = default_config_file, description="Config file to use"),
        DeclareLaunchArgument("port", default_value = "/dev/ttyUSB0", description="Port to use"),
        
        Node(
            package="riptide_gyro",
            executable="gyro_driver",
            name="riptide_gyro",
            output="screen",
            parameters=[
                LC("config_file"),
                { "gyro_port" : LC("port") }
            ]
        )
    ])
