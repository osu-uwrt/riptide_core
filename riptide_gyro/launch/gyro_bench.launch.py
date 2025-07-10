import os
from ament_index_python import get_package_share_directory
from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC

default_config_file = os.path.join(
    get_package_share_directory("riptide_gyro"),
    "config",
    "gyro_config.yaml"
)

gyro_launch_file = os.path.join(
    get_package_share_directory('riptide_gyro'),
    "launch", "gyro.launch.py"
)

imu_launch_file = os.path.join(
    get_package_share_directory('riptide_imu'),
    "launch", "imu.launch.py"
)

nav_launch_file = os.path.join(
    get_package_share_directory('riptide_hardware2'),
    "launch", "navigation.launch.py"
)

mapping_launch_file = os.path.join(
    get_package_share_directory('riptide_mapping2'),
    "launch", "mapping.launch.py"
)

def launch_imu(context, **kwargs):
    imu = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(imu_launch_file),
        launch_arguments=[
            ('robot', LC('robot')),
            ("imu_port", LC("imu_port"))
        ]
    )
    
    if LC("with_imu").perform(context) != "False":
        return [ imu ]

    return [ ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="talos", description="Name of the robot"),
        DeclareLaunchArgument("gyro_port", default_value="/dev/ttyUSB0", description="Port to use"),
        DeclareLaunchArgument("imu_port", default_value="/dev/ttyTHS0", description="IMU port. Only used if IMU is enabled"),
        DeclareLaunchArgument("with_imu", default_value="False", description="Optional enable switch for IMU"),
        DeclareLaunchArgument("config_file", default_value = default_config_file, description="Config file to use"),
        
        #navigation
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(nav_launch_file),
            launch_arguments = [
                ("robot", LC("robot"))
            ]
        ),
        
        #mapping (for map frame and easy viewing in rviz)
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(mapping_launch_file),
            launch_arguments = [
                ("robot", LC("robot"))
            ]
        ),
        
        #things that arent already namespaced
        GroupAction([
            PushRosNamespace(LC("robot")),
                        
            #fake hardware
            Node(
                package='riptide_hardware2',
                executable='fake_dvl.py',
                name='fake_dvl'
            ),
            
            #real hardware
            OpaqueFunction(function=launch_imu),
            
            #gyro driver
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(gyro_launch_file),
                launch_arguments = [
                    ("gyro_port", LC("gyro_port")),
                    ("config_file", LC("config_file"))
                ]
            )
        ], scoped=True)
    ])
