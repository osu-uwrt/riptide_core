import launch
import launch.actions
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration as LC
from launch.substitutions import PathJoinSubstitution
import os
import xacro
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch_ros.actions import Node, PushRosNamespace
import traceback

# evaluates LaunchConfigurations in context for use with xacro.process_file(). Returns a list of launch actions to be included in launch description
def evaluate_xacro(context, *args, **kwargs):
    robot = LC('robot').perform(context)
    debug = LC('debug').perform(context)

    modelPath = PathJoinSubstitution([
        get_package_share_directory('riptide_descriptions2'),
        'robots',
        robot + '.xacro'
    ]).perform(context)

    try:
        xacroData = xacro.process_file(modelPath,  mappings={'debug': debug, 'namespace': robot, 'inertial_reference_frame':'world'}).toxml()

        robot_state_publisher = Node(
            name = 'robot_state_publisher',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output = 'screen',
            arguments=['--ros-args', '--log-level', 'WARN'],
            parameters=[
                {'robot_description': xacroData},
                {'use_tf_static': True}
                ], # Use subst here
        )
        
        with open('/tmp/model.urdf', 'w') as urdf_file:
            urdf_file.write(xacroData)
        
        joint_state = Node(
            name="joint_state_publisher",
            package="joint_state_publisher",
            executable="joint_state_publisher",
            output="screen",
            arguments=['/tmp/model.urdf']
        )
        
        return [robot_state_publisher, joint_state]
    except Exception as ex:
        print()
        print("---------------------------------------------")
        print("COULD NOT OPEN ROBOT DESCRIPTION XACRO FILE")
        traceback.print_exc()
        print("---------------------------------------------")
        print()

    return []

def generate_launch_description():
    # declare the launch args to read for this file
    config = PathJoinSubstitution([
        get_package_share_directory('riptide_hardware2'),
        'cfg',
        LC("robot_ekf_config")
    ])

    return launch.LaunchDescription([ 
        # Read in the vehicle's namespace through the command line or use the default value one is not provided
        DeclareLaunchArgument(
            "robot", 
            default_value="tempest",
            description="Namespace of the vehicle",
        ),

        DeclareLaunchArgument(
            "debug", 
            default_value="False",
            description="enable xacro debug of the vehicle",
        ),

        DeclareLaunchArgument(
            "robot_ekf_config", 
            default_value=[LC("robot"), '_ekf.yaml'],
            description="EKF yaml file for the vehicle",
        ),

        GroupAction([
            PushRosNamespace(
                LC("robot")
            ),

            # Publish world and odom as same thing until we get SLAM
            # This is here so we can compare ground truth from sim to odom
            Node(
                name="odom_to_world_broadcaster",
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "world", "odom"]
            ),

            # start robot_localization Extended Kalman filter (EKF)
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_localization_node',
                output='screen',
                parameters=[
                    config,
                    {
                        'reset_on_time_jump': True,
                    }
                ]
            ),
                
            Node(
                package='riptide_hardware2',
                executable='depth_converter',
                name='depth_converter',
            ),

            # Publish robot model for Sensor locations
            OpaqueFunction(function=evaluate_xacro),
        ], scoped=True)
    ])
