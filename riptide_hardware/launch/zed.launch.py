from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, Shutdown
from launch_ros.actions import PushRosNamespace, ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression, LaunchConfiguration as LC
from ament_index_python import get_package_share_directory
import os



def generate_launch_description():

    # Define common configuration path
    zed_config_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        "config",
        "common_stereo.yaml"
    )


    # Paths for individual camera configurations

    # FFC Zed 2i
    zed2i_camera_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'config',
        'zed2i.yaml'
    )
    
    # FFC Zed X
    zedx_camera_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'config',
        'zedx.yaml'
    )
    
    # DFC Zed X Mini
    zedxm_camera_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'config',
        'zedxm.yaml'
    )
    
    # FFC Overrides
    ffc_config_path = os.path.join(
        get_package_share_directory('riptide_hardware2'),
        'cfg',
        'ffc_config.yaml'
    )

    # DFC Overrides
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
                            zed_config_path,
                            zedx_camera_path,
                            ffc_config_path,
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
                            zed_config_path,
                            zedxm_camera_path,
                            dfc_config_path,
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
            
            Node(
                package='riptide_hardware2',
                executable='picture_taker.py',
                name='picture_taker',
                output='screen',
                parameters=[
                    {"robot_namespace": LC("robot")},
                    {"camera_name": "ffc"},
                    {"save_stereo": True},
                    {"save_split": True}
                ]
            )
            
        ], scoped=True),


    ])