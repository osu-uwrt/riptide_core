from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("imu_port", default_value="/dev/ttyUSB0",
                              description="Exclusive serial port for the VN-100"),
        DeclareLaunchArgument("baud", default_value="460800"),
        DeclareLaunchArgument("config_file", default_value="",
                              description="Control Center XML to stage at startup"),
        DeclareLaunchArgument("demo", default_value="false",
                              description="Use generated data and disable sensor writes"),
        Node(
            package="riptide_imu",
            executable="sensor_config",
            name="vn_sensor_config",
            output="screen",
            parameters=[{
                "imu_port": LaunchConfiguration("imu_port"),
                "baud": ParameterValue(LaunchConfiguration("baud"), value_type=int),
                "config_file": LaunchConfiguration("config_file"),
                "demo": ParameterValue(LaunchConfiguration("demo"), value_type=bool),
            }],
        ),
    ])
