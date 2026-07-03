from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import PushRosNamespace, ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration as LC
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    zed_wrapper_share = get_package_share_directory("zed_wrapper")
    riptide_share = get_package_share_directory("riptide_hardware2")

    zed_common = os.path.join(zed_wrapper_share, "config", "common_stereo.yaml")
    zedxm = os.path.join(zed_wrapper_share, "config", "zedxm.yaml")

    ffc_cfg = os.path.join(riptide_share, "cfg", "ffc_config.yaml")
    dfc_cfg = os.path.join(riptide_share, "cfg", "dfc_config.yaml")

    ffc_node = ComposableNode(
        package="zed_components",
        plugin="stereolabs::ZedCamera",
        namespace="ffc",
        name="zed_node",
        parameters=[zed_common, zedxm, ffc_cfg],
    )

    dfc_node = ComposableNode(
        package="zed_components",
        plugin="stereolabs::ZedCamera",
        namespace="dfc",
        name="zed_node",
        parameters=[zed_common, zedxm, dfc_cfg],
    )

    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="tempest"),

        GroupAction([
            PushRosNamespace(LC("robot")),

            ComposableNodeContainer(
                name="zed_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container",
                output="screen",
                respawn=False, 
                composable_node_descriptions=[ffc_node, dfc_node],
            ),

            Node(
                package='riptide_hardware2',
                executable='picture_taker.py',
                name='picture_taker',
                output='screen',
                parameters=[
                    {"robot_namespace": LC("robot")},
                    {"camera_name": "ffc"},
                    {"subscription_enabled": True},
                    {"save_stereo": False},
                    {"save_split": True}
                ]
            )
        ], scoped=True),
    ])