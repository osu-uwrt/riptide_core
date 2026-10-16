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
    if mode not in ("replacement", "comparison", "baseline", "shadow"):
        raise RuntimeError(f"Unknown estimator mode: {mode}")

    own_share = get_package_share_directory("riptide_navigation")
    actions = []
    if mode != "shadow":
        hardware_navigation = os.path.join(
            get_package_share_directory("riptide_hardware2"), "launch", "navigation.launch.py")
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware_navigation),
            launch_arguments={"robot": robot, "ekf_enabled": str(mode != "replacement")}.items()))

    if mode != "baseline":
        parameters = [os.path.join(own_share, "config", "talos_ekf.yaml")]
        profile = LaunchConfiguration("ekf_profile").perform(context)
        profiles = {
            "default": None,
            "pool_conservative": "talos_ekf_pool_tuned.yaml",
            "pool_tracking": "talos_ekf_pool_tracking.yaml",
            "pool_consider": "talos_ekf_pool_consider.yaml",
        }
        if profile not in profiles:
            raise RuntimeError(f"Unknown ekf_profile: {profile}")
        candidate = LaunchConfiguration("use_candidate_tuning").perform(context).lower() == "true"
        if candidate and profile != "default":
            raise RuntimeError("Choose ekf_profile or use_candidate_tuning, not both")
        if profiles[profile]:
            parameters.append(os.path.join(own_share, "config", profiles[profile]))
        if candidate:
            parameters.append(os.path.join(own_share, "config", "talos_ekf_tuned.yaml"))
        parameters.extend([
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"publish_tf": mode == "replacement"},
        ])
        remappings = []
        if mode in ("comparison", "shadow"):
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
            description="replacement, comparison, baseline, or shadow (existing hardware/RL)"),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Use /clock for simulator or bag replay"),
        DeclareLaunchArgument(
            "ekf_profile", default_value="default",
            description="default, pool_conservative, pool_tracking, or pool_consider"),
        DeclareLaunchArgument(
            "use_candidate_tuning", default_value="false",
            description="Overlay the unaccepted candidate tuning file"),
        OpaqueFunction(function=_launch),
    ])
