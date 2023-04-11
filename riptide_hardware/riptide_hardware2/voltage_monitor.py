#!/usr/bin/env python3

import math
import rclpy
import socket
import yaml
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
import diagnostic_updater
from riptide_msgs2.msg import BatteryStatus, ElectricalReadings
from diagnostic_msgs.msg import DiagnosticStatus

from .common import ExpiringMessage

class BatteryVoltageTask(diagnostic_updater.DiagnosticTask):
    def __init__(self, side: str, battery_status_msg: ExpiringMessage, voltage_thresholds, current_thresholds):
        diagnostic_updater.DiagnosticTask.__init__(self, "{} Battery Voltage".format(side))

        self.side = side
        self._warning_voltage = float(voltage_thresholds["warn"])
        self._error_voltage = float(voltage_thresholds["thruster_cutoff"])
        self._warning_current = float(current_thresholds["warn"])
        self._error_current = float(current_thresholds["fuse"])

        self._battery_status_msg = battery_status_msg

    def run(self, stat):
        battery_status: 'BatteryStatus' = self._battery_status_msg.get_value()

        if battery_status is None:
            stat.summary(DiagnosticStatus.STALE, "No data available")
            return stat

        stat.add("SOC", "{:.0f}%".format(battery_status.soc))
        stat.add("Voltage", "{:.2f}V".format(battery_status.pack_voltage))
        stat.add("Current", "{:.2f}A".format(battery_status.pack_current))
        stat.add("Average Current", "{:.2f}A".format(battery_status.average_current))
        stat.add("Time to Discharge", "{:.0f} min".format(battery_status.time_to_dischg))
        stat.add("Serial", "{:04d}".format(battery_status.serial))
        stat.add("Cell Name", battery_status.cell_name)

        if battery_status.pack_current <= self._error_current:
            stat.summary(DiagnosticStatus.ERROR, "Battery ({:.2f}A) above battery fuse rating ({}A)".format(battery_status.pack_current, self._error_current))
        elif battery_status.pack_current <= self._warning_current:
            stat.summary(DiagnosticStatus.WARN, "Battery ({:.2f}A) above nominal current ({}A)".format(battery_status.pack_current, self._warning_current))
        elif battery_status.pack_voltage <= self._error_voltage:
            stat.summary(DiagnosticStatus.ERROR, "Battery ({:.2f}V) below thruster cutoff voltage ({}V)".format(battery_status.pack_voltage, self._error_voltage))
        elif battery_status.pack_voltage <= self._warning_voltage:
            stat.summary(DiagnosticStatus.ERROR, "Battery ({:.2f}V) below thruster cutoff voltage ({}V)".format(battery_status.pack_voltage, self._warning_voltage))
        else:
            stat.summary(DiagnosticStatus.OK, "{:d}% {:.2f}V {:.1f}A".format(battery_status.soc, battery_status.pack_voltage, battery_status.pack_current))

        return stat


class ThrusterCurrentTask(diagnostic_updater.DiagnosticTask):
    def __init__(self, electrical_readings_msg: ExpiringMessage, current_thresholds):
        diagnostic_updater.DiagnosticTask.__init__(self, "Thruster Current")

        self._warning_current = float(current_thresholds["warn"])
        self._error_current = float(current_thresholds["fuse"])

        self._electrical_readings_msg = electrical_readings_msg

    def generateThrusterList(self, thruster_list, thruster_currents):
        if len(thruster_list) == 1:
            message = "Thruster "
        else:
            message = "Thrusters "
        for i in range(len(thruster_list)):
            if i != 0:
                message += ", "
            thruster_id = thruster_list[i]
            message += "{} ({:.2f}A)".format(thruster_id+1, thruster_currents[thruster_id])
        return message

    def run(self, stat):
        electrical_reading = self._electrical_readings_msg.get_value()

        if electrical_reading is None:
            stat.summary(DiagnosticStatus.STALE, "No message available")
            return stat

        thruster_currents = electrical_reading.esc_current

        if any(math.isnan(x) for x in thruster_currents):
            stat.summary(DiagnosticStatus.ERROR, "Unable to read thruster currents")
        else:
            error_thrusters = []
            warning_thrusters = []
            zero_current_thrusters = []
            current_total = 0
            for i in range(len(thruster_currents)):
                extra_info = ""
                if thruster_currents[i] >= self._error_current:
                    error_thrusters.append(i)
                    extra_info = " [OVER FUSE]"
                elif thruster_currents[i] >= self._warning_current:
                    warning_thrusters.append(i)
                    extra_info = " [WARN]"
                elif thruster_currents[i] == 0:
                    zero_current_thrusters.append(i)
                    extra_info = " [OFF]"
                stat.add("Thruster {} Current".format(i+1), "{:.2f}A".format(thruster_currents[i]) + extra_info)
                current_total += thruster_currents[i]

            stat.add("Total Current", "{:.2f}A".format(current_total))

            if len(error_thrusters) > 0:
                error_message = self.generateThrusterList(error_thrusters, thruster_currents)
                stat.summary(DiagnosticStatus.ERROR, "{} above ESC fuse rating ({}A) (Total Current: {:.2f}A})".format(error_message, self._error_current, current_total))
            elif len(warning_thrusters) > 0:
                warning_message = self.generateThrusterList(warning_thrusters, thruster_currents)
                stat.summary(DiagnosticStatus.WARN, "{} above nominal ESC current ({}A) (Total Current: {:.2f}A)".format(warning_message, self._warning_current, current_total))
            else:
                stat.summary(DiagnosticStatus.OK, "Total Thruster Current: {:.2f}A".format(current_total))

        return stat


class VoltageMonitorTask(diagnostic_updater.DiagnosticTask):
    def __init__(self, electrical_readings_msg: ExpiringMessage, rail_thresholds, rail_name, reading_field):
        diagnostic_updater.DiagnosticTask.__init__(self, "{} Rail Voltage".format(rail_name))

        self.rail_name = rail_name
        self.reading_field = reading_field
        self._warn_voltage_min = float(rail_thresholds["warn_voltage_min"])
        self._warn_voltage_max = float(rail_thresholds["warn_voltage_max"])

        self._electrical_readings_msg = electrical_readings_msg

    def run(self, stat):
        electrical_reading = self._electrical_readings_msg.get_value()

        if electrical_reading is None:
            stat.summary(DiagnosticStatus.STALE, "No message available")
            return stat

        rail_voltage = getattr(electrical_reading, self.reading_field)

        if math.isnan(rail_voltage):
            stat.summary(DiagnosticStatus.ERROR, "Unable to read {} rail".format(self.rail_name))
        else:
            stat.add("{} Rail Voltage".format(self.rail_name), "{:.2f}V".format(rail_voltage))

            if rail_voltage <= self._warn_voltage_min:
                stat.summary(DiagnosticStatus.WARN, "Rail ({:.2f}V) under nominal voltage ({}V)".format(rail_voltage, self._warn_voltage_min))
            elif rail_voltage >= self._warn_voltage_max:
                stat.summary(DiagnosticStatus.WARN, "Rail ({:.2f}V) above nominal voltage ({}V)".format(rail_voltage, self._warn_voltage_max))
            else:
                stat.summary(DiagnosticStatus.OK, "Rail at {:.2f}V".format(rail_voltage))

        return stat


class VoltageMonitor:
    def electrical_state_cb(self, msg):
        self.electrical_readings_msg.update_value(msg)

    def battery_state_cb(self, msg: 'BatteryStatus'):
        if msg.detect == BatteryStatus.DETECT_STBD:
            self.stbd_batt_status.update_value(msg)
        elif msg.detect == BatteryStatus.DETECT_PORT:
            self.port_batt_status.update_value(msg)

    def run(self):
        hostname = socket.gethostname()
        rclpy.init()
        node = rclpy.create_node('voltage_monitor')
        node.declare_parameter('robot', rclpy.Parameter.Type.STRING)
        node.declare_parameter('diag_thresholds_file', rclpy.Parameter.Type.STRING)

        current_robot = node.get_parameter('robot').value

        # Load config file
        with open(node.get_parameter('diag_thresholds_file').value, 'r') as stream:
            thresholds_file = yaml.safe_load(stream)
        message_lifetime = thresholds_file["ros_message_lifetime"]
        thresholds = thresholds_file["volt_cur_thresholds"]

        # Subscribe to messages
        self.electrical_readings_msg = ExpiringMessage(node.get_clock(), message_lifetime)
        node.create_subscription(ElectricalReadings, "state/electrical", self.electrical_state_cb, qos_profile_sensor_data)

        self.stbd_batt_status = ExpiringMessage(node.get_clock(), message_lifetime)
        self.port_batt_status = ExpiringMessage(node.get_clock(), message_lifetime)
        node.create_subscription(BatteryStatus, "state/battery", self.battery_state_cb, qos_profile_system_default)

        # Create diagnostics updater
        updater = diagnostic_updater.Updater(node)
        updater.setHardwareID(hostname)

        # updater.add(BatteryVoltageTask(self.electrical_readings_msg, thresholds["battery_volt"]))
        if current_robot == "puddles":
            updater.add(ThrusterCurrentTask(self.electrical_readings_msg, thresholds["puddles_thruster_current"]))
        else:
            updater.add(BatteryVoltageTask("Port", self.port_batt_status, thresholds["battery_volt"], thresholds["battery_current"]))
            updater.add(BatteryVoltageTask("Starboard", self.stbd_batt_status, thresholds["battery_volt"], thresholds["battery_current"]))

        updater.add(VoltageMonitorTask(self.electrical_readings_msg, thresholds["three_volt"], "3.3V", "three_volt_voltage"))
        updater.add(VoltageMonitorTask(self.electrical_readings_msg, thresholds["five_volt"], "5V", "five_volt_voltage"))
        updater.add(VoltageMonitorTask(self.electrical_readings_msg, thresholds["twelve_volt"], "15V", "twelve_volt_voltage"))
        updater.add(VoltageMonitorTask(self.electrical_readings_msg, thresholds["balanced_volt"], "V+", "balanced_voltage"))

        updater.force_update()

        rclpy.spin(node, None)

    @staticmethod
    def main():
        monitor = VoltageMonitor()
        monitor.run()

if __name__ == '__main__':
    VoltageMonitor.main()