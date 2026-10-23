from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("connection_mode", default_value="ros",
                              description="Initial connection mode: ros, serial, or demo"),
        DeclareLaunchArgument("remote_namespace", default_value="",
                              description="Initial robot namespace; empty enables discovery"),
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
                "connection_mode": LaunchConfiguration("connection_mode"),
                "remote_namespace": LaunchConfiguration("remote_namespace"),
                "baud": ParameterValue(LaunchConfiguration("baud"), value_type=int),
                "config_file": LaunchConfiguration("config_file"),
                "demo": ParameterValue(LaunchConfiguration("demo"), value_type=bool),
            }],
        ),
    ])
