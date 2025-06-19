from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, Shutdown
from launch_ros.actions import PushRosNamespace, ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression, LaunchConfiguration as LC
from ament_index_python import get_package_share_directory
import os

# Define common configuration file paths
zed_config_path = os.path.join(
    get_package_share_directory('riptide_hardware2'),
    "cfg",
    "zed_common.yaml"
)

def generate_launch_description():
    # Paths for individual camera configurations
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
    
    tank_config_path = os.path.join(
        get_package_share_directory('riptide_hardware2'),
        'cfg',
        'tank_ffc_config.yaml'
    )
    
    zed_compression_path = os.path.join(
        get_package_share_directory('riptide_hardware2'),
        'cfg',
        'zed_compression.yaml'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            name="robot",
            default_value="tempest",
            description="name of the robot"
        ),
        # Group actions under the robot namespace
        GroupAction([
            PushRosNamespace(LC("robot")),

            ComposableNodeContainer(
                name="ffc_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container",
                output="screen",
                respawn=True,
                composable_node_descriptions=[
                    # First ZED Node (FFC)
                    ComposableNode(
                        package="zed_components",
                        plugin="stereolabs::ZedCamera",
                        namespace="ffc",
                        name="zed_node",
                        parameters=[
                            zed2i_camera_path,
                            zed_config_path,
                            ffc_config_path,
                            zed_compression_path,
                            {"general.camera_name": "talos/ffc"},
                        ]
                    ),                
                ],
                condition=IfCondition(
                    PythonExpression(["'", LC("robot"), "' == 'talos'"]))
            ),
            
            ComposableNodeContainer(
                name="dfc_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container",
                output="screen",
                respawn=True,
                composable_node_descriptions=[
                    # Second ZED Node (DFC)
                    ComposableNode(
                        package="zed_components",
                        plugin="stereolabs::ZedCamera",
                        namespace="dfc",
                        name="zed_node",
                        parameters=[
                            zedxm_camera_path,
                            zed_config_path,
                            dfc_config_path,
                            zed_compression_path,
                            {"general.camera_name": "talos/dfc"},
                        ]
                    ),
                ],
                condition=IfCondition(
                    PythonExpression(["'", LC("robot"), "' == 'talos'"]))
            ),
            
            ComposableNodeContainer(
                name="ffc_container",
                namespace="/",
                package="rclcpp_components",
                executable="component_container",
                output="screen",
                respawn=True,
                composable_node_descriptions=[
                    # The tank camera
                    ComposableNode(
                        package="zed_components",
                        plugin="stereolabs::ZedCamera",
                        namespace="ffc",
                        name="zed_node",
                        parameters=[
                            # zedxm_camera_path,
                            zed_config_path,
                            tank_config_path,
                            # zed_compression_path,
                            {"general.camera_name": "liltank/ffc"},
                            {"mapping.clicked_point_topic": "/clicked_point"},
                        ]
                    ),
                ],
                condition=IfCondition(
                    PythonExpression(["'", LC("robot"), "' == 'liltank'"]))
            ),
            
            # Disabled for now as both are on by default
            # # Launch the ZedManager node
            # Node(
            #     package='riptide_hardware2',
            #     executable='zed_manager.py',
            #     name='ZedManager',
            #     output='screen'
            # )
            
        ], scoped=True),


    ])
