from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration as LC
from launch.substitutions import TextSubstitution as TS
import os

imu_config_file = os.path.join(
    get_package_share_directory('riptide_hardware2'),
    "cfg", "vectornav_imu_config.yaml"
)

# Vectornav
start_vectornav_cmd = Node(
    package='vectornav', 
    executable='vectornav',
    output='screen',
    parameters=[
        imu_config_file,
        {
            'frame_id': LC("imu_frame_id"),
            'port': LC("serial_port")
        }
    ],
    respawn=True,
)

start_vectornav_sensor_msgs_cmd = Node(
    package='vectornav', 
    executable='vn_sensor_msgs',
    output='screen',
    parameters=[
        imu_config_file,
    ],
    respawn=True,
)

robot_arg = DeclareLaunchArgument('robot', default_value="tempest", description="Name of the vehicle")
serial_arg = DeclareLaunchArgument('serial_port', default_value="/dev/ttyTHS0")
frame_id_arg = DeclareLaunchArgument('imu_frame_id', default_value=[LC('robot'), "/imu_link"])

def generate_launch_description():
    # Create the launch description and populate
    return LaunchDescription([
        # launch args
        robot_arg,
        serial_arg,
        frame_id_arg,
        
        # print the startup info
        # LogInfo(msg=LC("frame_id")),
        # LogInfo(msg=["Serial port: ", LC("serial_port")]),
        
        # start the nodes
        start_vectornav_cmd,
        start_vectornav_sensor_msgs_cmd
    ])