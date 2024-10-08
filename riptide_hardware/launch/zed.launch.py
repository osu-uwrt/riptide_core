from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration as LC, PythonExpression
from launch.conditions import IfCondition
from ament_index_python import get_package_share_directory
import os

zed_launch_file = os.path.join(get_package_share_directory(
    "zed_wrapper"), "launch", "zed_camera.launch.py")
zed_config_path = os.path.join(get_package_share_directory('riptide_hardware2'), "cfg", "zed_common.yaml")

def generate_launch_description():
    zed_camera_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'config',
        'zed2i.yaml'
    )
    
    # Create the launch description and populate
    return LaunchDescription([
        DeclareLaunchArgument(
            name = "robot",
            default_value = "tempest",
            description = "name of the robot"
        ),
        
        DeclareLaunchArgument(
            "zed_name",
            default_value=[LC("robot"), "/zed"]
        ),
        
        Node(
            package='zed_wrapper',
            executable='zed_wrapper',
            namespace="zed",
            name='zed_node',
            output='screen',
            parameters=[
                # YAML files
                zed_config_path,  # Common parameters
                zed_camera_path,  # Camera related parameters
                # Overriding
                {
                    'general.camera_name': "talos/zed",
                    'general.camera_model': "zed2i",
                    'pos_tracking.publish_tf': False,
                    'pos_tracking.publish_map_tf': False,
                    'sensors.publish_imu_tf': False,
                    # 'general.svo_file': "/home/ros/svos/practice_sat.svo"
                },
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
