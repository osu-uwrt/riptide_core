from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_fake_ivc(context, **kwargs):
    robot = LaunchConfiguration("robot").perform(context)
    client = LaunchConfiguration("client_robot").perform(context)
    fail_rate = LaunchConfiguration("failure_rate").perform(context)
    
    if robot == client:
        print("NOT launching because robot and client are set to the same robot")
        return []
    
    return [
        Node(
            package="riptide_hardware2",
            executable="fake_ivc.py",
            name="fake_ivc",
            parameters=[
                {
                    "host_robot_name": robot,
                    "remote_robot_name": client,
                    "failure_rate": float(fail_rate)
                }
            ]
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot",
            default_value="talos",
            description="Name of the host (this) robot"
        ),
        
        DeclareLaunchArgument(
            "client_robot",
            default_value="liltank",
            description="Name of the remote robot"
        ),
        
        DeclareLaunchArgument(
            "failure_rate",
            default_value="0.2",
            description="Failure rate of the IVC system"
        ),
        
        OpaqueFunction(function=launch_fake_ivc)
    ])
