from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import PushRosNamespace, Node
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
    get_package_share_directory('riptide_imu'),
    "launch", "imu.launch.py"
)

gyro_launch_file = os.path.join(
    get_package_share_directory('riptide_gyro'),
    "launch", "gyro.launch.py"
)

apriltag_launch_file = os.path.join(
    get_package_share_directory('riptide_hardware2'),
    "launch", "apriltag.launch.py"
)

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value="tempest",
                              description="Name of the vehicle"),
        
        GroupAction([
            PushRosNamespace(
                LC("robot")
            ),
            # Zenoh router launch goes first to allow discovery
            Node(
               package='rmw_zenoh_cpp',
               executable='rmw_zenohd',
               name='zenoh_router',
               output='screen', 
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
                    ('robot', LC('robot'))
                ]
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(gyro_launch_file),
                launch_arguments=[
                    ('robot', LC('robot'))
                ],
                condition=IfCondition(
                    PythonExpression(["'", LC("robot"), "' == 'talos'"])
                )
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(apriltag_launch_file),
                launch_arguments=[
                    ('robot', LC('robot')),
                ]
            ),
            Node(
                package='riptide_hardware2',
                executable='simple_actuator_interface.py',
                name='simple_actuator_interface',
                output='screen',
            ),
            # launched here instead of in imu launch because this launch can reasonably assume it is running on jetson HW
            Node(
                package='riptide_hardware2',
                executable='imu_power_cycle.py',
                name='imu_power_cycle',
                output='screen',
                parameters=[
                    os.path.join(
                        get_package_share_directory("riptide_hardware2"),
                        "cfg",
                        "imu_pwr_cycle.yaml"
                    )
                ]
            ),
            Node(
                package='riptide_hardware2',
                executable='pressure_monitor.py',
                name='pressure_monitor',
                output='screen',
                parameters=[{"robot":LC('robot')}]
            )
        ], scoped=True)
    ])
