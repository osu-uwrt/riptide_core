#!/usr/bin/env python3

import rclpy
import diagnostic_updater
import socket
import yaml
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from datetime import timedelta
from riptide_msgs2.msg import BatteryStatus, FirmwareStatus
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticStatus

from .common import ExpiringMessage, Mk2Board


class FirmwareMonitor(diagnostic_updater.DiagnosticTask):
    # Fill this out with each firmware's error descriptions
    ERROR_DESCRIPTIONS = []

    def __init__(self, node: 'rclpy.Node', message_lifetime, asserting_kill: 'ExpiringMessage', board: 'Mk2Board', *, kill_switch_special=False):
        diagnostic_updater.DiagnosticTask.__init__(self, board.friendly_name)

        self._asserting_kill: 'ExpiringMessage' = asserting_kill
        self._firmware_status = ExpiringMessage(node.get_clock(), message_lifetime)
        self.kill_switch_special = kill_switch_special
        self.board = board

    def gen_fault_string(self, fault_bits):
        fault_list = []
        fault_num = 0
        while fault_bits != 0:
            if (fault_bits & 1) != 0:
                if fault_num < len(self.ERROR_DESCRIPTIONS):
                    fault_list.append(self.ERROR_DESCRIPTIONS[fault_num])
                else:
                    fault_list.append("UNKNOWN_ERR_{0}".format(fault_num))
            fault_num += 1
            fault_bits >>= 1
        return ", ".join(fault_list)

    def refresh_potential_msg(self, msg: 'FirmwareStatus'):
        if msg.client_id != self.board.client_id or msg.bus_id != self.board.bus_id:
            return

        self._firmware_status.update_value(msg)

    def run(self, stat: 'diagnostic_updater.DiagnosticStatusWrapper'):
        msg: 'FirmwareStatus' = self._firmware_status.get_value()

        if msg == None:
            stat.summary(DiagnosticStatus.ERROR, "Not Connected")
            return stat

        uptime = timedelta(milliseconds=msg.uptime_ms)
        faults = msg.faults

        # Release type lookup
        release_type_string = "Unknown ({})".format(msg.version_release_type)
        if msg.version_release_type == 0:
            release_type_string = "Debug"
        elif msg.version_release_type == 1:
            release_type_string = "Dev"
        elif msg.version_release_type == 2:
            release_type_string = "Clean"
        elif msg.version_release_type == 3:
            release_type_string = "Tagged"

        stat.add("Faults", self.gen_fault_string(faults))
        stat.add("Uptime", str(uptime))
        stat.add("Kill Switch Status", "Killed" if msg.kill_switches_asserting_kill else "Enabled")
        stat.add("Kill Switch Timed Out", "Yes" if msg.kill_switches_timed_out else "No")
        stat.add("Board Name", str(msg.board_name))
        stat.add("Bus ID", str(msg.bus_id))
        stat.add("Client ID", str(msg.client_id))
        stat.add("FW Version", "{}.{}".format(msg.version_major, msg.version_minor))
        stat.add("Release Type", release_type_string)

        # Unexpected Status
        if msg.board_name != self.board.board_name:
            stat.summary(DiagnosticStatus.ERROR, "Unexpected Board")

        # Error State
        if not self.kill_switch_special and msg.kill_switches_enabled != 1 and msg.kill_switches_needs_update != 1:
            stat.summary(DiagnosticStatus.ERROR, "Unexpected Kill Switch Configuration")
        elif not self.kill_switch_special and msg.kill_switches_timed_out != 0 and self._asserting_kill is not None:
            stat.summary(DiagnosticStatus.ERROR, "Unexpected Kill Switch Timeout")
        elif not self.kill_switch_special and self._asserting_kill is not None and msg.kill_switches_asserting_kill != self._asserting_kill.get_value():
            stat.summary(DiagnosticStatus.ERROR, "Local kill switch does not match global state")
        elif faults != 0:
            stat.summary(DiagnosticStatus.ERROR, "Fault Code " + str(self.gen_fault_string(faults)))

        # Warning States
        elif not self.kill_switch_special and msg.kill_switches_timed_out != 0:
            # Warn that the kill switch has timed out, but since we checked that asserting kill timed out,
            # it's not a fault
            stat.summary(DiagnosticStatus.WARN, "Kill Switch Timeout")
        elif msg.version_release_type == 0:
            # Lowest priority warning, anything else will show over this
            # But still here to make sure poeple don't try to run debug firmware and get bit by extra latency
            stat.summary(DiagnosticStatus.WARN, "Running Debug Firmware")

        # Normal States
        else:
            stat.summary(DiagnosticStatus.OK, "Connected")

        return stat


class ESCBoardMonitor(FirmwareMonitor):
    ERROR_DESCRIPTIONS = [
        "FAULT_WATCHDOG_RESET",
        "FAULT_WATCHDOG_WARNING",
        "FAULT_CAN_INTERNAL_ERROR",
        "FAULT_ROS_ERROR",
        "FAULT_TIMER_MISSED",
        "FAULT_ROS_BAD_COMMAND",
        "FAULT_THRUSTER_TIMEOUT",
        "FAULT_RAW_MODE",
    ]

    def __init__(self, node: 'rclpy.Node', message_lifetime, asserting_kill: 'ExpiringMessage', board_num: int):
        assert board_num == 0 or board_num == 1, "Invalid ESC Board Num"
        super().__init__(node, message_lifetime, asserting_kill, Mk2Board.ESC_BOARD_0 if board_num == 0 else Mk2Board.ESC_BOARD_1)

class PowerBoardMonitor(FirmwareMonitor):
    ERROR_DESCRIPTIONS = [
        "FAULT_WATCHDOG_RESET",
        "FAULT_WATCHDOG_WARNING",
        "FAULT_CAN_INTERNAL_ERROR",
        "FAULT_ROS_ERROR",
        "FAULT_TIMER_MISSED",
        "FAULT_ROS_BAD_COMMAND",
        "FAULT_ADC_ERROR",
        "FAULT_SHT41_ERROR",
    ]

    def __init__(self, node: 'rclpy.Node', message_lifetime, asserting_kill: 'ExpiringMessage'):
        super().__init__(node, message_lifetime, asserting_kill, Mk2Board.POWER_BOARD, kill_switch_special=True)

class CameraCageBBMonitor(FirmwareMonitor):
    ERROR_DESCRIPTIONS = [
        "FAULT_WATCHDOG_RESET",
        "FAULT_WATCHDOG_WARNING",
        "FAULT_CAN_INTERNAL_ERROR",
        "FAULT_ROS_ERROR",
        "FAULT_TIMER_MISSED",
        "FAULT_ROS_BAD_COMMAND",
        "FAULT_DEPTH_INIT_ERROR",
        "FAULT_DEPTH_ERROR",
        "FAULT_SHT41_ERROR",
    ]

    def __init__(self, node: 'rclpy.Node', message_lifetime, asserting_kill: 'ExpiringMessage'):
        super().__init__(node, message_lifetime, asserting_kill, Mk2Board.CAMERA_CAGE_BB)

class ActuatorBoardMonitor(FirmwareMonitor):
    ERROR_DESCRIPTIONS = [
        "FAULT_WATCHDOG_RESET",
        "FAULT_WATCHDOG_WARNING",
        "FAULT_CAN_INTERNAL_ERROR",
        "FAULT_ROS_ERROR",
        "FAULT_TIMER_MISSED",
        "FAULT_ACTUATOR_FAILURE",
        "FAULT_ACTUATOR_UNPLUGGED",
    ]

    def __init__(self, node: 'rclpy.Node', message_lifetime, asserting_kill: 'ExpiringMessage'):
        super().__init__(node, message_lifetime, asserting_kill, Mk2Board.ACTUATOR_BOARD)

class SmartBatteryMonitor(FirmwareMonitor):
    ERROR_DESCRIPTIONS = [
        "FAULT_WATCHDOG_RESET",
        "FAULT_WATCHDOG_WARNING",
        "FAULT_CAN_INTERNAL_ERROR",
        "FAULT_ROS_ERROR",
        "FAULT_TIMER_MISSED",
        "FAULT_BQ40_ERROR",
        "FAULT_SHT41_ERROR"
    ]

    cachedSerialNum = None

    def refresh_potential_msg(self, msg: 'FirmwareStatus'):
        if msg.client_id != self.cachedSerialNum or msg.bus_id != self.board.bus_id:
            return  # TODO: Make not has hardcoded for serial to bus id selection

        self._firmware_status.update_value(msg)

    def battery_state_cb(self, msg: 'BatteryStatus'):
        if msg.detect == self.board.client_id:
            self.cachedSerialNum = msg.serial

    def __init__(self, node: 'rclpy.Node', message_lifetime, asserting_kill: 'ExpiringMessage', board_num: int):
        super().__init__(node, message_lifetime, asserting_kill, Mk2Board.SBH_MCU_PORT if board_num == 0 else Mk2Board.SBH_MCU_STBD)
        node.create_subscription(BatteryStatus, "state/battery", self.battery_state_cb, qos_profile_system_default)

class PuddlesCoproMonitor(FirmwareMonitor):
    ERROR_DESCRIPTIONS = [
        "FAULT_WATCHDOG_RESET",
        "FAULT_WATCHDOG_WARNING",
        "FAULT_ROS_ERROR",
        "FAULT_TIMER_MISSED",
        "FAULT_ROS_BAD_COMMAND",
        "FAULT_THRUSTER_TIMEOUT",
        "FAULT_DEPTH_INIT_ERROR",
        "FAULT_DEPTH_ERROR",
        "FAULT_ADC_ERROR"
    ]

    def __init__(self, node: 'rclpy.Node', message_lifetime, asserting_kill: 'ExpiringMessage'):
        super().__init__(node, message_lifetime, asserting_kill, Mk2Board.PUDDLES_BACKPLANE, kill_switch_special=True)


class FirmwareMonitor:
    def firmware_state_cb(self, msg):
        for listener in self.firmware_state_listeners:
            listener.refresh_potential_msg(msg)

    def kill_state_cb(self, msg: 'Bool'):
        self.asserting_kill_msg.update_value(int(msg.data))

    def run(self):
        hostname = socket.gethostname()
        rclpy.init()
        node = rclpy.create_node('firmware_monitor')
        node.declare_parameter('robot', rclpy.Parameter.Type.STRING)
        node.declare_parameter('diag_thresholds_file', rclpy.Parameter.Type.STRING)

        current_robot = node.get_parameter('robot').value

        # Load config file
        with open(node.get_parameter('diag_thresholds_file').value, 'r') as stream:
            thresholds_file = yaml.safe_load(stream)
        message_lifetime = float(thresholds_file["ros_message_lifetime"])

        # Subscribe to messages
        self.asserting_kill_msg = ExpiringMessage(node.get_clock(), message_lifetime)
        node.create_subscription(FirmwareStatus, "state/firmware", self.firmware_state_cb, qos_profile_sensor_data)
        node.create_subscription(Bool, "state/kill", self.kill_state_cb, qos_profile_sensor_data)

        # Create diagnostics updater
        updater = diagnostic_updater.Updater(node)
        updater.setHardwareID(hostname)

        if current_robot == "talos":
            self.firmware_state_listeners = [
                PowerBoardMonitor(node, message_lifetime, self.asserting_kill_msg),
                ESCBoardMonitor(node, message_lifetime, self.asserting_kill_msg, 0),
                ESCBoardMonitor(node, message_lifetime, self.asserting_kill_msg, 1),
                CameraCageBBMonitor(node, message_lifetime, self.asserting_kill_msg),
                ActuatorBoardMonitor(node, message_lifetime, self.asserting_kill_msg),
                SmartBatteryMonitor(node, message_lifetime, self.asserting_kill_msg, 0),
                SmartBatteryMonitor(node, message_lifetime, self.asserting_kill_msg, 1)
            ]
        else:
            self.firmware_state_listeners = [
                PuddlesCoproMonitor(node, message_lifetime, self.asserting_kill_msg),
            ]

        for listener in self.firmware_state_listeners:
            updater.add(listener)

        updater.force_update()

        rclpy.spin(node, None)

    @staticmethod
    def main():
        monitor = FirmwareMonitor()
        monitor.run()


if __name__ == '__main__':
    FirmwareMonitor.main()
