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
from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
    PackageNotFoundError,
)
import importlib.util
import os
import tempfile

import yaml


def _launch(context):
    robot = LaunchConfiguration("robot").perform(context)
    mode = LaunchConfiguration("mode").perform(context)
    if mode != "replacement":
        raise RuntimeError(f"Unsupported mode '{mode}': the model is the vehicle's estimator")

    own_share = get_package_share_directory("riptide_navigation")
    try:
        get_package_prefix("talos_navigation")
    except PackageNotFoundError:
        raise RuntimeError("The generated talos_navigation package is missing: build it with "
                           "riptide_simulink (model_manager.py build talos_navigation)") from None

    # the rest of the hardware navigation stack, without its own EKF
    hardware_navigation = os.path.join(
        get_package_share_directory("riptide_hardware2"), "launch", "navigation.launch.py")

    spec = importlib.util.spec_from_file_location(
        "navigation_config", os.path.join(own_share, "launch", "navigation_config.py"))
    navigation_config = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(navigation_config)
    schema = navigation_config.load_schema(
        os.path.join(own_share, "config", "parameter_schema.json"))
    config = navigation_config.load_config(
        LaunchConfiguration("navigation_config").perform(context) or
        os.path.join(own_share, "config", "talos_navigation.yaml"))
    config["publish_tf"] = True
    errors = navigation_config.validate(config, schema)
    if errors:
        raise RuntimeError("Invalid navigation config: " + "; ".join(errors))
    model, overseer = navigation_config.ros_parameters(config, schema)

    folder = tempfile.mkdtemp(prefix="talos_navigation_")
    model_file = os.path.join(folder, "model.yaml")
    with open(model_file, "w") as out:
        yaml.safe_dump({"/**": {"ros__parameters": model}}, out)
    overseer.update({
        "model_node": "/" + robot + "/talos_navigation",
        "model_parameter_file": model_file,
        "diagnostic_keys_file": os.path.join(own_share, "config", "diagnostic_keys.json"),
    })

    # Only an explicit use_sim_time:= overrides a parent SetParameter (sim). Not
    # declared, since a declared default leaks into every later include.
    use_sim_time = context.launch_configurations.get("use_sim_time", "")
    common = [{"use_sim_time": use_sim_time.lower() == "true"}] if use_sim_time else []

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware_navigation),
            launch_arguments={"robot": robot, "ekf_enabled": "false"}.items()),
        GroupAction([
            PushRosNamespace(robot),
            Node(package="talos_navigation", executable="talos_navigation",
                 name="talos_navigation", output="screen", parameters=[model_file] + common),
            Node(package="riptide_navigation", executable="talos_navigation_overseer",
                 name="talos_navigation_overseer", output="screen",
                 parameters=[overseer] + common),
        ]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot", default_value="talos",
            description="Vehicle namespace and hardware-description selection"),
        DeclareLaunchArgument(
            "mode", default_value="replacement",
            description="Kept for existing launch files; replacement is the only mode"),
        DeclareLaunchArgument(
            "navigation_config", default_value="",
            description="Nested navigation config; empty uses the installed one"),
        OpaqueFunction(function=_launch),
    ])
