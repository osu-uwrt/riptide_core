# Copyright 2026 OSU Underwater Robotics Team
# SPDX-License-Identifier: BSD-3-Clause
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os


def _launch(context):
    mode = LaunchConfiguration("mode").perform(context)
    robot = LaunchConfiguration("robot").perform(context)
    if mode not in ("replacement", "comparison", "baseline"):
        raise RuntimeError(f"mode must be replacement, comparison, or baseline, not {mode}")

    hardware_navigation = os.path.join(
        get_package_share_directory("riptide_hardware2"), "launch", "navigation.launch.py")
    own_share = get_package_share_directory("riptide_navigation")
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hardware_navigation),
        launch_arguments={"robot": robot, "ekf_enabled": str(mode != "replacement")}.items())]

    if mode != "baseline":
        parameters = [os.path.join(own_share, "config", "talos_ekf.yaml")]
        if LaunchConfiguration("use_candidate_tuning").perform(context).lower() == "true":
            parameters.append(os.path.join(own_share, "config", "talos_ekf_tuned.yaml"))
        parameters.extend([
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"publish_tf": mode == "replacement"},
        ])
        remappings = []
        if mode == "comparison":
            remappings = [
                ("odometry/filtered", "simulink/odometry/filtered"),
                ("accel/filtered", "simulink/accel/filtered"),
                ("set_pose", "simulink/set_pose"),
            ]
        actions.append(GroupAction([
            PushRosNamespace(robot),
            Node(package="riptide_navigation", executable="talos_ekf_node",
                 name="talos_ekf_node", output="screen", parameters=parameters,
                 remappings=remappings),
        ]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot", default_value="talos",
            description="Vehicle namespace and hardware-description selection"),
        DeclareLaunchArgument(
            "mode", default_value="replacement",
            description="replacement, comparison, or baseline estimator mode"),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Use /clock for simulator or bag replay"),
        DeclareLaunchArgument(
            "use_candidate_tuning", default_value="false",
            description="Overlay the unaccepted candidate tuning file"),
        OpaqueFunction(function=_launch),
    ])
