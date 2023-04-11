from launch.launch_description import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC
from ament_index_python import get_package_share_directory
import os

zed_launch_file = os.path.join(get_package_share_directory(
    "zed_wrapper"), "launch", "zed2i.launch.py")
vision_model_path = os.path.join(get_package_share_directory(
    "riptide_hardware2"), "weights", "best.onnx")


def generate_launch_description():
    # Create the launch description and populate
    return LaunchDescription([
        # start the zed
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(zed_launch_file),
            launch_arguments=[
                #'onnx_weights_path': vision_model_path
                ('publish_map_tf', False),
                ('base_frame', [LC('robot'), '/zed2i_base_link'])
            ]
        ),

        # start the zed pose converter
        Node(
            package='riptide_hardware2',
            executable='pose_converter',
            name='pose_converter',
            output='screen',
            respawn=True
        )
    ])
