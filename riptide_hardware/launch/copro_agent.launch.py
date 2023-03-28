from launch.launch_description import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration as LC
from launch.substitutions import PythonExpression
from launch.conditions import IfCondition


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='copro_agent',
            output='screen',
            arguments=['can', '-D', 'can0', '-v4'], ## change to -v4 for actual logs
            respawn=True,
            condition=IfCondition(
                PythonExpression(["'", LC("robot"), "' == 'talos'"])
            ),
        ),

        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='copro_agent',
            output='screen',
            arguments=['udp4', '--port', '8888', '-v4'], ## change to -v4 for actual logs
            respawn=True,
            condition=IfCondition(
                PythonExpression(["'", LC("robot"), "' == 'puddles'"])
            ),
        ),

        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='external_agent',
            output='screen',
            arguments=['can', '-D', 'can1', '-v4'], ## change to -v4 for actual logs
            respawn=True
        )
    ])