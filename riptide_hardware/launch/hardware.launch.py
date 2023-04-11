import launch
from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import GroupAction, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration as LC
from launch.substitutions import PythonExpression
from launch.conditions.if_condition import IfCondition

import os

copro_agent_launch_file = os.path.join(
    get_package_share_directory('riptide_hardware2'),
    "launch", "copro_agent.launch.py",
)

diagnostics_launch_file = os.path.join(
    get_package_share_directory('riptide_hardware2'),
    "launch", "diagnostics.launch.py"
)

dvl_launch_file = os.path.join(
    get_package_share_directory('riptide_hardware2'),
    "launch", "dvl.launch.py"
)

imu_launch_file = os.path.join(
    get_package_share_directory('riptide_hardware2'),
    "launch", "imu.launch.py"
)

zed_launch_file = os.path.join(
    get_package_share_directory('riptide_hardware2'),
    "launch", "zed.launch.py"
)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value="tempest",
                              description="Name of the vehicle"),

        GroupAction([
            PushRosNamespace(
                LC("robot")
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(copro_agent_launch_file),
                launch_arguments=[
                    ('robot', LC('robot')),
                ]
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(diagnostics_launch_file),
                launch_arguments=[
                    ('robot', LC('robot')),
                ]
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(dvl_launch_file),
                launch_arguments=[
                    ('robot', LC('robot')),
                ],
                condition=IfCondition(
                    PythonExpression(["'", LC("robot"), "' == 'talos'"])
                )
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(imu_launch_file),
                launch_arguments=[
                    ('robot', LC('robot')),
                ]
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(zed_launch_file),
            )
        ], scoped=True)
    ])
