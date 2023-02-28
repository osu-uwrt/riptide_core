from launch.launch_description import LaunchDescription
from launch_ros.actions import Node
import launch.actions

copro_agent = Node(
    package='micro_ros_agent',
    executable='micro_ros_agent',
    name='copro_agent',
    output='screen',
    arguments=['can', '-D', 'can0', '-v4'], ## change to -v4 for actual logs
    respawn=True
)


def generate_launch_description():
    return LaunchDescription([
        launch.actions.GroupAction([
            copro_agent,
        ], scoped=True),
    ])