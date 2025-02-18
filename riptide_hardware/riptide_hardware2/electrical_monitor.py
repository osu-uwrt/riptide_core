#!/usr/bin/env python3

import rclpy
import diagnostic_updater
import socket
import yaml
from rclpy.qos import qos_profile_sensor_data
from datetime import timedelta
from riptide_msgs2.msg import DshotPartialTelemetry, DshotSingleTelemetry, DynamixelStatus, FirmwareStatus, KillSwitchReport
from std_msgs.msg import Bool, Float32
from diagnostic_msgs.msg import DiagnosticStatus

from common import ExpiringMessage, Mk2Board

class WaterTemperatureTask(diagnostic_updater.DiagnosticTask):
    def __init__(self, node: 'rclpy.Node', message_lifetime, warn_temp_below, warn_temp_above):
        diagnostic_updater.DiagnosticTask.__init__(self, "Water Temperature")

        self._warn_temp_below = float(warn_temp_below)
        self._warn_temp_above = float(warn_temp_above)
        self._water_temp = ExpiringMessage(node.get_clock(), message_lifetime)

        node.create_subscription(Float32, "state/depth/temp", self.get_water_temp, qos_profile_sensor_data)

    def get_water_temp(self, msg: 'Float32'):
        self._water_temp.update_value(msg.data)

    def run(self, stat: 'diagnostic_updater.DiagnosticStatusWrapper'):
        water_temp = self._water_temp.get_value()

        if water_temp is None:
            stat.summary(DiagnosticStatus.ERROR, "No reading available")
            return stat

        stat.add("Temperature", "{:.3f}C".format(water_temp))

        if water_temp >= self._warn_temp_above:
            stat.summary(DiagnosticStatus.WARN, "Temp {:.2f}C above nominal water temp {:.1f}C".format(water_temp, self._warn_temp_above))
        elif water_temp <= self._warn_temp_below:
            stat.summary(DiagnosticStatus.WARN, "Temp {:.2f}C below nominal water temp {:.1f}C".format(water_temp, self._warn_temp_below))
        else:
            stat.summary(DiagnosticStatus.OK, "{:.2f}C".format(water_temp))

        return stat


class ESCMonitorTask(diagnostic_updater.DiagnosticTask):
    def __init__(self, node: 'rclpy.Node', message_lifetime, warn_current, error_current):
        diagnostic_updater.DiagnosticTask.__init__(self, "ESC Status")

        self.warn_current = float(warn_current)
        self.error_current = float(error_current)
        self._esc_0_status = ExpiringMessage(node.get_clock(), message_lifetime)
        self._esc_1_status = ExpiringMessage(node.get_clock(), message_lifetime)
        node.create_subscription(DshotPartialTelemetry, "state/thrusters/telemetry", self.esc_telemetry_cb, qos_profile_sensor_data)

    def esc_telemetry_cb(self, msg: 'DshotPartialTelemetry'):
        if msg.start_thruster_num == 0:
            self._esc_0_status.update_value(msg)
        elif msg.start_thruster_num == 4:
            self._esc_1_status.update_value(msg)

    def run(self, stat: 'diagnostic_updater.DiagnosticStatusWrapper'):
        esc_0_status: 'DshotPartialTelemetry' = self._esc_0_status.get_value()
        esc_1_status: 'DshotPartialTelemetry' = self._esc_1_status.get_value()

        if esc_0_status is None and esc_1_status is None:
            stat.summary(DiagnosticStatus.ERROR, "ESC Boards 0 and 1 Not Connected")
            return stat
        elif esc_0_status is None:
            stat.summary(DiagnosticStatus.ERROR, "ESC Board 0 Not Connected")
            return stat
        elif esc_1_status is None:
            stat.summary(DiagnosticStatus.ERROR, "ESC Board 1 Not Connected")
            return stat

        stat.add("Board 0 VCC", "{:.2f}V".format(esc_0_status.vcc_voltage))
        stat.add("Board 1 VCC", "{:.2f}V".format(esc_1_status.vcc_voltage))
        stat.add("Board 0 Powered", "On" if esc_0_status.escs_powered else "Off")
        stat.add("Board 1 Powered", "On" if esc_1_status.escs_powered else "Off")

        not_present = []
        rpm_present = False
        for i in range(4):
            value: 'DshotSingleTelemetry' = esc_0_status.esc_telemetry[i]
            if not value.esc_online:
                not_present.append(str(i+1))
            stat.add("ESC {} Speed".format(i+1), "{}RPM".format(value.rpm))
            stat.add("ESC {} Voltage".format(i+1), "{:.2f}V".format(value.voltage))
            stat.add("ESC {} Current".format(i+1), "{:.2f}A".format(value.current))
            stat.add("ESC {} Consumption".format(i+1), "{:.3f}Ah".format(value.consumption_ah))
            stat.add("ESC {} Temperature".format(i+1), "{}C".format(value.temperature_c))
            if value.rpm != 0:
                rpm_present = True

        for i in range(4):
            value: 'DshotSingleTelemetry' = esc_1_status.esc_telemetry[i]
            i += 4
            if not value.esc_online:
                not_present.append(str(i+1))
            stat.add("ESC {} Speed".format(i+1), "{}RPM".format(value.rpm))
            stat.add("ESC {} Voltage".format(i+1), "{:.2f}V".format(value.voltage))
            stat.add("ESC {} Current".format(i+1), "{:.2f}A".format(value.current))
            stat.add("ESC {} Consumption".format(i+1), "{:.3f}Ah".format(value.consumption_ah))
            stat.add("ESC {} Temperature".format(i+1), "{}C".format(value.temperature_c))
            if value.rpm != 0:
                rpm_present = True

        if not esc_0_status.escs_powered and not esc_1_status.escs_powered:
            stat.summary(DiagnosticStatus.OK, "ESCs Off")
        elif not esc_0_status.escs_powered or not esc_1_status.escs_powered:
            stat.summary(DiagnosticStatus.ERROR, "ESC Board {} Powered while {} Off".format(0 if esc_0_status.escs_powered else 1, 1 if esc_0_status.escs_powered else 0))
        elif len(not_present) != 0:
            stat.summary(DiagnosticStatus.ERROR, "ESC{} {} Offline".format("s" if len(not_present) > 0 else "", ", ".join(not_present)))
        elif rpm_present:
            stat.summary(DiagnosticStatus.OK, "ESCs Moving")
        else:
            stat.summary(DiagnosticStatus.OK, "ESCs Ready")

        return stat

class KillSwitchTask(diagnostic_updater.DiagnosticTask):
    def __init__(self, node: 'rclpy.Node', message_lifetime, asserting_kill, authority_board: 'Mk2Board'):
        diagnostic_updater.DiagnosticTask.__init__(self, "Kill Switch")

        self._firmware_status = ExpiringMessage(node.get_clock(), message_lifetime)
        self._asserting_kill: 'ExpiringMessage' = asserting_kill
        self.authority_board: 'Mk2Board' = authority_board

    def refresh_potential_msg(self, msg: 'FirmwareStatus'):
        if msg.client_id != self.authority_board.client_id or msg.bus_id != self.authority_board.bus_id:
            return

        self._firmware_status.update_value(msg)

    def lookup_kill_switch(self, switch_id):
        if switch_id == KillSwitchReport.KILL_SWITCH_PHYSICAL:
            return "Physical"
        elif switch_id == KillSwitchReport.KILL_SWITCH_RQT_CONTROLLER:
            return "RQT Controller"
        elif switch_id == KillSwitchReport.KILL_SWITCH_TOPSIDE_BUTTON:
            return "Topside Button"
        elif switch_id == KillSwitchReport.KILL_SWITCH_DEBUG:
            return "Debug"
        else:
            return "Unknown ID {0}".format(switch_id)

    def make_bitwise_kill_switch_list(self, switch_bits):
        switch_id = 0
        switch_list = []
        while switch_bits != 0:
            if (switch_bits & 1) != 0:
                switch_list.append(self.lookup_kill_switch(switch_id))
            switch_bits >>= 1
            switch_id += 1
        return switch_list

    def run(self, stat: 'diagnostic_updater.DiagnosticStatusWrapper'):
        asserting_kill = self._asserting_kill.get_value()
        firmware_status: 'FirmwareStatus' = self._firmware_status.get_value()

        if asserting_kill is None or firmware_status is None:
            stat.summary(DiagnosticStatus.STALE, "No data available from " + self.authority_board.friendly_name)
            return stat

        kill_switch_inserted = not asserting_kill
        kill_switches_enabled = firmware_status.kill_switches_enabled
        kill_switches_asserting_kill = firmware_status.kill_switches_asserting_kill
        kill_switches_needs_update = firmware_status.kill_switches_needs_update
        kill_switches_timed_out = firmware_status.kill_switches_timed_out

        stat.add("Kill Switch State", "Run" if kill_switch_inserted else "Killed")
        stat.add("Active Kill Switches", "\n".join(self.make_bitwise_kill_switch_list(kill_switches_enabled)))
        stat.add("Switches Requiring Update", "\n".join(self.make_bitwise_kill_switch_list(kill_switches_needs_update)))
        stat.add("Switches Asserting Kill", "\n".join(self.make_bitwise_kill_switch_list(kill_switches_asserting_kill)))
        stat.add("Switches Timed Out", "\n".join(self.make_bitwise_kill_switch_list(kill_switches_timed_out)))

        if kill_switch_inserted != (not bool(kill_switches_asserting_kill | kill_switches_timed_out)):
            stat.summary(DiagnosticStatus.ERROR, "Kill state does not match expected state!")
        elif kill_switches_timed_out != 0:
            stat.summary(DiagnosticStatus.WARN, "{} kill switch timed out".format(", ".join(self.make_bitwise_kill_switch_list(kill_switches_timed_out))))
        elif kill_switch_inserted:
            stat.summary(DiagnosticStatus.OK, "Inserted")
        elif (kill_switches_asserting_kill & (1<<KillSwitchReport.KILL_SWITCH_PHYSICAL)) != 0:
            stat.summary(DiagnosticStatus.OK, "Removed")
        elif kill_switches_asserting_kill != 0:
            stat.summary(DiagnosticStatus.OK, "{} switch asserting kill".format(", ".join(self.make_bitwise_kill_switch_list(kill_switches_asserting_kill))))
        else:
            stat.summary(DiagnosticStatus.STALE, "Bad State!")

        return stat


class DynamixelMonitor(diagnostic_updater.DiagnosticTask):
    def __init__(self, node: 'rclpy.Node', message_lifetime, name, id):
        diagnostic_updater.DiagnosticTask.__init__(self, name)
        self.target_id = id

        self._dynamixel_status = ExpiringMessage(node.get_clock(), message_lifetime)
        node.create_subscription(DynamixelStatus, "state/actuator/dynamixel", self.dynamixel_status_cb, qos_profile_sensor_data)

    def dynamixel_status_cb(self, msg: 'DynamixelStatus'):
        if msg.id == self.target_id:
            self._dynamixel_status.update_value(msg)

    def run(self, stat: 'diagnostic_updater.DiagnosticStatusWrapper'):
        dynamixel_status: 'DynamixelStatus' = self._dynamixel_status.get_value()

        if dynamixel_status is None:
            stat.summary(DiagnosticStatus.ERROR, "Dynamixel {} Not Connected".format(self.target_id))
            return stat

        stat.add("Model", str(dynamixel_status.model))
        stat.add("Firmware Version", str(dynamixel_status.fw_version))
        stat.add("Hardware Err Status", str(dynamixel_status.hw_err_status))
        stat.add("Torque Enable", "Enabled" if dynamixel_status.torque_enable else "Disabled")
        stat.add("Moving", "Yes" if dynamixel_status.moving else "No")
        stat.add("Voltage", "%.1f V" % (dynamixel_status.voltage / 10))
        stat.add("Temperature", "%d C" % dynamixel_status.temperature)
        stat.add("Position", str(dynamixel_status.position))
        stat.add("Load", str(dynamixel_status.load))

        if dynamixel_status.hw_err_status != 0:
            stat.summary(DiagnosticStatus.ERROR, "Hardware Error {}".format(dynamixel_status.hw_err_status))
        else:
            stat.summary(DiagnosticStatus.OK, "Okay")

        return stat


class TorpedoMarkerMonitor(DynamixelMonitor):
    def __init__(self, node, message_lifetime):
        super().__init__(node, message_lifetime, "Torpedo/Marker", 2)

class ElectricalMonitor:
    def firmware_state_cb(self, msg):
        for listener in self.firmware_state_listeners:
            listener.refresh_potential_msg(msg)

    def kill_state_cb(self, msg: 'Bool'):
        self.asserting_kill_msg.update_value(int(msg.data))

    def run(self):
        hostname = socket.gethostname()
        rclpy.init()
        node = rclpy.create_node('electrical_monitor')
        node.declare_parameter('robot', rclpy.Parameter.Type.STRING)
        node.declare_parameter('diag_thresholds_file', rclpy.Parameter.Type.STRING)

        current_robot = node.get_parameter('robot').value

        # Load config file
        with open(node.get_parameter('diag_thresholds_file').value, 'r') as stream:
            thresholds_file = yaml.safe_load(stream)
        message_lifetime = float(thresholds_file["ros_message_lifetime"])
        thresholds = thresholds_file["electrical_monitor_thresholds"]

        # Subscribe to messages
        self.asserting_kill_msg = ExpiringMessage(node.get_clock(), message_lifetime)
        node.create_subscription(FirmwareStatus, "state/firmware", self.firmware_state_cb, qos_profile_sensor_data)
        node.create_subscription(Bool, "state/kill", self.kill_state_cb, qos_profile_sensor_data)

        # Create diagnostics updater
        updater = diagnostic_updater.Updater(node)
        updater.setHardwareID(hostname)

        if current_robot == "talos":
            self.firmware_state_listeners = [
                KillSwitchTask(node, message_lifetime, self.asserting_kill_msg, Mk2Board.POWER_BOARD),
            ]

            updater.add(TorpedoMarkerMonitor(node, message_lifetime))
            updater.add(ESCMonitorTask(node, message_lifetime, thresholds_file["volt_cur_thresholds"]["talos_thruster_current"]["warn"], thresholds_file["volt_cur_thresholds"]["talos_thruster_current"]["fuse"]))
        else:
            self.firmware_state_listeners = [
                KillSwitchTask(node, message_lifetime, self.asserting_kill_msg, Mk2Board.PUDDLES_BACKPLANE),
            ]

        updater.add(WaterTemperatureTask(node, message_lifetime, thresholds["water_low_warn_temp"], thresholds["water_high_warn_temp"]))
        #updater.add(RobotTemperatureTask(self.robot_state_msg, self.firmware_state_msg, thresholds["temp_over_target_warn"]))
        for listener in self.firmware_state_listeners:
            updater.add(listener)

        updater.force_update()

        rclpy.spin(node, None)

    @staticmethod
    def main():
        monitor = ElectricalMonitor()
        monitor.run()


if __name__ == '__main__':
    ElectricalMonitor.main()
