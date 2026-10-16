#!/usr/bin/env python3
"""Checks the public launch arguments for local and remote configuration."""

import pathlib
import sys
import unittest


class LaunchInterfaceTest(unittest.TestCase):

    def test_configuration_defaults_to_ros_without_serial_path_argument(self):
        package = pathlib.Path(sys.argv[1])
        source = (package / "launch" / "sensor_config.launch.py").read_text(
            encoding="utf-8")
        self.assertIn('DeclareLaunchArgument("connection_mode", default_value="ros"',
                      source)
        self.assertIn('DeclareLaunchArgument("remote_namespace"', source)
        self.assertNotIn('DeclareLaunchArgument("imu_port"', source)

    def test_driver_keeps_imu_port_argument(self):
        package = pathlib.Path(sys.argv[1])
        source = (package / "launch" / "imu.launch.py").read_text(
            encoding="utf-8")
        self.assertIn('DeclareLaunchArgument(name="imu_port"', source)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
