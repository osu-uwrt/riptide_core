from launch.launch_description import LaunchDescription
from launch_ros.actions import Node

copro_agent = Node(
    package='micro_ros_agent',
    executable='micro_ros_agent',
    name='copro_agent',
    output='screen',
    arguments=['can', '-D', 'can0', '-v4'], ## change to -v4 for actual logs
    respawn=True
)
depth_agent = Node(
    package='micro_ros_agent',
    executable='micro_ros_agent',
    name='depth_agent',
    output='screen',
    arguments=['serial', '--dev', '/dev/uwrt_depth', '-v4'], ## change to -v4 for actual logs
    respawn=True
)

def generate_launch_description():
    return LaunchDescription([
        copro_agent,
        depth_agent
    ])