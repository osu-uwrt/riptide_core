from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import PushRosNamespace, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration as LC
from ament_index_python import get_package_share_directory
import os

zed_launch_file = os.path.join(get_package_share_directory(
    "zed_wrapper"), "launch", "zed_camera.launch.py")
zed_config_path = os.path.join(get_package_share_directory('riptide_hardware2'), "cfg", "zed_common.yaml")
zed_compression_path = os.path.join(get_package_share_directory('riptide_hardware2'), "cfg", "zed_compression.yaml")

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
        
        GroupAction([
            PushRosNamespace(
                LC("robot")
            ),
        
            ComposableNodeContainer(
                name="zed_container",
                namespace="ffc",
                package='rclcpp_components',
                executable="component_container",
                # arguments=['--use_multi_threaded_executor','--ros-args', '--log-level', 'info'],
                output='screen',
                composable_node_descriptions=[
                    ComposableNode(
                        package='zed_components',
                        plugin='stereolabs::ZedCamera',
                        namespace="ffc",
                        name='zed_node',
                        parameters=[
                            # YAML files
                            zed_config_path,  # Common parameters
                            # zed_compression_path,
                            zed2i_camera_path,  # Camera related parameters
                            # Overriding
                            {
                                'general.camera_name': "talos/ffc",
                                'general.camera_model': "zed2i",
                                'pos_tracking.publish_tf': False,
                                'pos_tracking.publish_map_tf': False,
                                'sensors.publish_imu_tf': False,
                                'sensors.sensors_image_sync': False, # If using imu use this
                                'general.optional_opencv_calibration_file': "/home/ros/zed_cals/ffc_calibration3.yaml",
                                'debug.debug_common': False,
                                'debug.debug_point_cloud': False
                                # 'general.svo_file': "/home/ros/svos/practice_sat.svo"
                            },
                        ]
                    ),
                ]
            ),
            
            ComposableNodeContainer(
                name="zed_container",
                namespace="dfc",
                package='rclcpp_components',
                executable="component_container",
                # arguments=['--use_multi_threaded_executor','--ros-args', '--log-level', 'info'],
                output='screen',
                composable_node_descriptions=[
                                    ComposableNode(
                        package='zed_components',
                        plugin='stereolabs::ZedCamera',
                        namespace="dfc",
                        name='zed_node',
                        parameters=[
                            # YAML files
                            zed_config_path,  # Common parameters
                            # zed_compression_path,
                            zedxm_camera_path,  # Camera related parameters
                            # Overriding
                            {
                                'general.camera_name': "talos/dfc",
                                'general.camera_model': "zedxm",
                                'pos_tracking.publish_tf': False,
                                'pos_tracking.publish_map_tf': False,
                                'sensors.publish_imu_tf': False,
                                'sensors.sensors_image_sync': False, # If using imu use this
                                'general.optional_opencv_calibration_file': "/home/ros/zed_cals/zed_dfc_calibration2.yaml",
                                'debug.debug_common': False,
                                'debug.debug_point_cloud': False
                                # 'general.svo_file': "/home/ros/svos/practice_sat.svo"
                            },
                        ]
                    )
                ]
            ),
        ], scoped=True)
    ])
