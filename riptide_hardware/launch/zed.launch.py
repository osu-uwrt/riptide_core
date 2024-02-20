from launch.launch_description import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC, PythonExpression
from launch.conditions import IfCondition
from ament_index_python import get_package_share_directory
import os

zed_launch_file = os.path.join(get_package_share_directory(
    "zed_wrapper"), "launch", "zed_camera.launch.py")
vision_model_path = os.path.join(get_package_share_directory(
    "riptide_hardware2"), "weights", "best.onnx")
zed_config_path = os.path.join(get_package_share_directory('riptide_hardware2'), "cfg", "zed_common.yaml")

def generate_launch_description():
    # Create the launch description and populate
    return LaunchDescription([
        DeclareLaunchArgument(
            name = "robot",
            default_value = "tempest",
            description = "name of the robot"
        ),

        # start the zed
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(zed_launch_file),
            launch_arguments=[
                ('config', zed_config_path),
                ('camera_model', 'zed2i'),
                ('publish_tf', "false"),
                ('publish_map_tf', "false"),
            ]
        ),

        # start the zed pose converter
        Node(
            package='riptide_hardware2',
            executable='pose_converter',
            name='pose_converter',
            output='screen',
            respawn=True,
            condition=IfCondition(
                PythonExpression(["'", LC("robot"), "' == 'puddles'"])
            )
        )
    ])
