import os
import xacro
import traceback
import launch
import launch.actions
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch.substitutions import LaunchConfiguration as LC
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch_ros.actions import Node, PushRosNamespace


def get_zed_description(zed_name, zed_type, robot, context, debug):
    zed_model_path = PathJoinSubstitution([
        get_package_share_directory('zed_wrapper'),
        'urdf',
        'zed_descr.urdf.xacro'
    ]).perform(context)

    zed_description_data = xacro.process_file(zed_model_path,  mappings={
        'debug': debug,
        'namespace': robot,
        'inertial_reference_frame':'world',
        'camera_name': f"{robot}/{zed_name}",
        'camera_model': zed_type}
    ).toxml()
    
    zed_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=f"{zed_name}",
        name='zed_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': zed_description_data},
            {'use_tf_static': True}
        ]
    )
    
    return zed_state_publisher


# evaluates LaunchConfigurations in context for use with xacro.process_file(). Returns a list of launch actions to be included in launch description
def evaluate_xacro(context, *args, **kwargs):
    robot = LC('robot').perform(context)
    debug = LC('debug').perform(context)

    robot_model_path = PathJoinSubstitution([
        get_package_share_directory('riptide_descriptions2'),
        'robots',
        robot + '.xacro'
    ]).perform(context)

    try:
        #
        # robot state publisher
        #
        robot_description_data = xacro.process_file(robot_model_path,  mappings={'debug': debug, 'namespace': robot, 'inertial_reference_frame':'world'}).toxml()

        robot_state_publisher = Node(
            name = 'robot_state_publisher',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output = 'screen',
            arguments=['--ros-args', '--log-level', 'WARN'],
            parameters=[
                {'robot_description': robot_description_data},
                {'use_tf_static': True}
            ], # Use subst here
        )
        
        with open('/tmp/model.urdf', 'w') as urdf_file:
            urdf_file.write(robot_description_data)
        
        joint_state = Node(
            name="joint_state_publisher",
            package="joint_state_publisher",
            executable="joint_state_publisher",
            output="screen",
            arguments=['/tmp/model.urdf']
        )
        
        nodes = [robot_state_publisher, joint_state]
        
        #
        # zed state publisher
        #
        try:
            if robot == "talos":
                nodes.append(get_zed_description("ffc", "zed2i", robot, context, debug))
                nodes.append(get_zed_description("dfc", "zedxm", robot, context, debug))

            if robot == "liltank":
                nodes.append(get_zed_description("ffc", "zedxm", robot, context, debug))
        except PackageNotFoundError:
            print("zed_wrapper not found. Launching without zed TF")
        
        return nodes
    
    except Exception as ex:
        print()
        print("---------------------------------------------")
        print("COULD NOT OPEN ROBOT DESCRIPTION OR ZED XACRO FILE")
        print(ex)
        traceback.print_exc()
        print("---------------------------------------------")
        print()

    return []


def launch_ekf(context, *args, **kwargs):
    launch_items = []
    tag_odom_enabled = LC("tag_odom_enabled").perform(context) == 'True'
    
    if LC("ekf_enabled").perform(context) == "True":
        robot = LC("robot").perform(context)
        ekf_config_name = f"{robot}_ekf.yaml" if not tag_odom_enabled else f"{robot}_ekf_tag_odom.yaml"
        
        config = os.path.join(
            get_package_share_directory('riptide_hardware2'),
            'cfg',
            ekf_config_name
        )
                
        # start robot_localization Extended Kalman filter (EKF)
        launch_items.append(
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
            )
        )
        
        if tag_odom_enabled:
            launch_items.append(
                Node(
                    package='riptide_hardware2',
                    executable='tag_odom.py',
                    name='tag_odom',
                    output='screen'
                )
            )
        
    return launch_items


def generate_launch_description():
    # declare the launch args to read for this file
    

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
            "ekf_enabled",
            default_value="True",
            description="Enable EKF to estimate robot odometry"
        ),
        
        DeclareLaunchArgument(
            "tag_odom_enabled",
            default_value="False",
            description="Enable navigation using the apriltag"
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

            Node(
                package='riptide_hardware2',
                executable='depth_converter.py',
                name='depth_converter',
            ),
            
            # start ekf
            OpaqueFunction(function=launch_ekf),

            # Publish robot model for Sensor locations
            OpaqueFunction(function=evaluate_xacro),
        ], scoped=True)
    ])
