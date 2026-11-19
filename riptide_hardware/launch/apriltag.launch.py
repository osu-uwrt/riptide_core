from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions.composable_node_container import ComposableNode, ComposableNodeContainer

cfg_36h11 = {
    "family": "36h11",
    "size": 0.508,
    "max_hamming": 0,
    "z_up": True
}

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        name="robot",
        default_value="tempest",
        description="name of the robot"
    ))
    # The robot's ZED publishes the left camera raw. The nereus simulator publishes only its rgb stream, JPEG
    # compressed: image_transport:=compressed camera:=rgb.
    ld.add_action(DeclareLaunchArgument(
        name="image_transport",
        default_value="raw",
        description="raw, or compressed (decoded in the node from <image>/compressed)"
    ))
    ld.add_action(DeclareLaunchArgument(
        name="camera",
        default_value="left",
        description="ffc ZED stream: left (robot) or rgb (simulator)"
    ))
    camera = LaunchConfiguration("camera")

    ld.add_action(ComposableNodeContainer(
        name='tag_container',
        namespace="apriltag",
        package='rclcpp_components',
        executable='component_container',
        arguments=['--ros-args', '--log-level', 'info'],
        composable_node_descriptions=[
            ComposableNode(
                name='apriltag_36h11',
                package='apriltag_ros', plugin='AprilTagNode',
                remappings=[
                    ("image_rect", ["ffc/zed_node/", camera, "/image_rect_color"]),
                    # the compressed path subscribes to image_rect/compressed by name, and ROS remaps whole names
                    ("image_rect/compressed", ["ffc/zed_node/", camera, "/image_rect_color/compressed"]),
                    ("camera_info", ["ffc/zed_node/", camera, "/camera_info"]),
                ],
                parameters=[cfg_36h11, {"image_transport": LaunchConfiguration("image_transport")}],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        ],
        output='screen'
    ))

    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='surface_frame_node',
        arguments=["0", "0.4572", "0", "0", "-1.5707", "-1.5707",
                   "tag36h11:0", "estimated_origin_frame"]
    ))

    return ld
