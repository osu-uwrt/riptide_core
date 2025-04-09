from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, Shutdown
from launch_ros.actions import PushRosNamespace, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration as LC
from ament_index_python import get_package_share_directory
import os

zed_launch_file = os.path.join(get_package_share_directory(
    "zed_wrapper"), "launch", "zed_camera.launch.py")
zed_config_path = os.path.join(get_package_share_directory('riptide_hardware2'), "cfg", "zed_common.yaml")

def generate_launch_description():
    zed2i_camera_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'config',
        'zed2i.yaml'
    )
    
    zedxm_camera_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'config',
        'zedxm.yaml'
    )
    
    ffc_config_path = os.path.join(
        get_package_share_directory('riptide_hardware2'),
        'cfg',
        'ffc_config.yaml'
    )
    
    dfc_config_path = os.path.join(
        get_package_share_directory('riptide_hardware2'),
        'cfg',
        'dfc_config.yaml'
    )
    
    # Create the launch description and populate
    return LaunchDescription([
        DeclareLaunchArgument(
            name = "robot",
            default_value = "tempest",
            description = "name of the robot"
        ),
        
        GroupAction([
            PushRosNamespace(
                LC("robot")
            ),
        
            ComposableNodeContainer(
                name="zed_container",
                namespace="ffc",
                package='rclcpp_components',
                executable="component_container",
                output='screen',
                on_exit=Shutdown(), # Zombie slayer
                respawn=True,
                composable_node_descriptions=[
                    ComposableNode(
                        package='zed_components',
                        plugin='stereolabs::ZedCamera',
                        namespace="ffc",
                        name='zed_node',
                        parameters=[
                            zed2i_camera_path,
                            zed_config_path,
                            ffc_config_path,      
                            {'general.camera_name': "talos/ffc"},
                        ]
                    ),
                ]
            ),
            
            ComposableNodeContainer(
                name="zed_container",
                namespace="dfc",
                package='rclcpp_components',
                executable="component_container",
                output='screen',
                on_exit=Shutdown(), # Zombie slayer
                respawn=True,
                composable_node_descriptions=[
                    ComposableNode(
                        package='zed_components',
                        plugin='stereolabs::ZedCamera',
                        namespace="dfc",
                        name='zed_node',
                        parameters=[
                            zedxm_camera_path,
                            zed_config_path,
                            dfc_config_path,
                            {'general.camera_name': "talos/dfc"}
                        ]
                    )
                ]
            ),
        ], scoped=True)
    ])