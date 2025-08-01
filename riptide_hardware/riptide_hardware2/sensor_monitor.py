#!/usr/bin/env python3

import rclpy
import socket
import yaml
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from diagnostic_msgs.msg import DiagnosticStatus
from nortek_dvl_msgs.msg import DvlStatus
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
from riptide_msgs2.msg import Depth, GyroStatus
from diagnostic_updater import DiagnosticStatusWrapper, DiagnosticTask, Updater

from common import ExpiringMessage

class DVLSensorTask(DiagnosticTask):
    def __init__(self, node: 'rclpy.Node', msg_lifetime):
        DiagnosticTask.__init__(self, "DVL Sensor")

        self._dvl_status = ExpiringMessage(node.get_clock(), msg_lifetime)
        self._dvl_twist = ExpiringMessage(node.get_clock(), msg_lifetime)

        node.create_subscription(DvlStatus, 'dvl/status', self.dvl_status_callback, qos_profile_sensor_data)
        node.create_subscription(TwistWithCovarianceStamped, 'dvl_twist', self.dvl_twist_callback, qos_profile_sensor_data)

    def dvl_status_callback(self, msg):
        self._dvl_status.update_value(msg)

    def dvl_twist_callback(self, msg):
        self._dvl_twist.update_value(True)

    def run(self, stat: 'DiagnosticStatusWrapper'):
        dvl_status: 'DvlStatus' = self._dvl_status.get_value()
        dvl_twist: 'TwistWithCovarianceStamped' = self._dvl_twist.get_value()

        if dvl_status is None:
            stat.summary(DiagnosticStatus.ERROR, "Not Connected")
        else:
            stat.add("B1 velocity valid", str(dvl_status.b1_vel_valid))
            stat.add("B2 velocity valid", str(dvl_status.b2_vel_valid))
            stat.add("B3 velocity valid", str(dvl_status.b3_vel_valid))
            stat.add("B4 velocity valid", str(dvl_status.b4_vel_valid))
            velocity_ok_count = int(dvl_status.b1_vel_valid) + int(dvl_status.b2_vel_valid) + int(dvl_status.b3_vel_valid) + int(dvl_status.b4_vel_valid)

            stat.add("B1 distance valid", str(dvl_status.b1_dist_valid))
            stat.add("B2 distance valid", str(dvl_status.b2_dist_valid))
            stat.add("B3 distance valid", str(dvl_status.b3_dist_valid))
            stat.add("B4 distance valid", str(dvl_status.b4_dist_valid))
            distance_ok_count = int(dvl_status.b1_dist_valid) + int(dvl_status.b2_dist_valid) + int(dvl_status.b3_dist_valid) + int(dvl_status.b4_dist_valid)

            stat.add("B1 FOM valid", str(dvl_status.b1_fom_valid))
            stat.add("B2 FOM valid", str(dvl_status.b2_fom_valid))
            stat.add("B3 FOM valid", str(dvl_status.b3_fom_valid))
            stat.add("B4 FOM valid", str(dvl_status.b4_fom_valid))
            fom_ok_count = int(dvl_status.b1_fom_valid) + int(dvl_status.b2_fom_valid) + int(dvl_status.b3_fom_valid) + int(dvl_status.b4_fom_valid)

            stat.add("X velocity valid", str(dvl_status.x_vel_valid))
            stat.add("Y velocity valid", str(dvl_status.y_vel_valid))
            stat.add("Z1 velocity valid", str(dvl_status.z1_vel_valid))
            stat.add("Z2 velocity valid", str(dvl_status.z2_vel_valid))
            xy2z_vel_ok_count = int(dvl_status.x_vel_valid) + int(dvl_status.y_vel_valid) + int(dvl_status.z1_vel_valid) + int(dvl_status.z2_vel_valid)

            stat.add("X FOM valid", str(dvl_status.x_fom_valid))
            stat.add("Y FOM valid", str(dvl_status.y_fom_valid))
            stat.add("Z1 FOM valid", str(dvl_status.z1_fom_valid))
            stat.add("Z2 FOM valid", str(dvl_status.z2_fom_valid))
            xy2z_fom_ok_count = int(dvl_status.x_fom_valid) + int(dvl_status.y_fom_valid) + int(dvl_status.z1_fom_valid) + int(dvl_status.z2_fom_valid)

            if dvl_twist is None:
                stat.summary(DiagnosticStatus.WARN, "Unable to get reading")
            else:
                stat.summary(DiagnosticStatus.OK, "OK ({0}/4 vel OK) ({1}/4 dist OK) ({2}/4 fom OK) ({3}/4 xy2z vel OK) ({4}/4 xy2z fom OK)".format(velocity_ok_count, distance_ok_count, fom_ok_count, xy2z_vel_ok_count, xy2z_fom_ok_count))

        return stat

class IMUSensorTask(DiagnosticTask):
    def __init__(self, node: 'rclpy.Node', msg_lifetime):
        DiagnosticTask.__init__(self, "IMU Sensor")

        self._imu_status = ExpiringMessage(node.get_clock(), msg_lifetime)

        node.create_subscription(Imu, 'vectornav/imu', self.imu_callback, qos_profile_sensor_data)

    def imu_callback(self, msg):
        self._imu_status.update_value(True)

    def run(self, stat: 'DiagnosticStatusWrapper'):
        imu_status = self._imu_status.get_value()

        if imu_status is None:
            stat.summary(DiagnosticStatus.ERROR, "Not Connected")
        else:
            stat.summary(DiagnosticStatus.OK, "OK")

        return stat

class DepthSensorTask(DiagnosticTask):
    def __init__(self, node: 'rclpy.Node', msg_lifetime, max_nominal):
        DiagnosticTask.__init__(self, "Depth Sensor")

        self.max_nominal = max_nominal

        self._depth_status = ExpiringMessage(node.get_clock(), msg_lifetime)
        node.create_subscription(Depth, 'state/depth/raw', self.depth_callback, qos_profile_sensor_data)

    def depth_callback(self, msg: 'Depth'):
        self._depth_status.update_value(msg.depth)

    def run(self, stat: 'DiagnosticStatusWrapper'):
        depth_raw = self._depth_status.get_value()

        if depth_raw is None:
            stat.summary(DiagnosticStatus.ERROR, "Not Connected")
            return stat

        if depth_raw > self.max_nominal:
            stat.summary(DiagnosticStatus.WARN, "Depth ({:.2f}) abnormally high".format(depth_raw))
        else:
            stat.summary(DiagnosticStatus.OK, "Connected")

        return stat

class FOGSensorTask(DiagnosticTask):
    def __init__(self, node: 'rclpy.Node', msg_lifetime):
        DiagnosticTask.__init__(self, "Gyro")
        self._fog_status = ExpiringMessage(node.get_clock(), msg_lifetime)
        node.create_subscription(GyroStatus, "gyro/status", self.fog_callback, qos_profile_system_default)
    
    def fog_callback(self, msg: 'GyroStatus'):
        self._fog_status.update_value(msg)
        
    def run(self, stat: 'DiagnosticStatusWrapper'):
        fog_stat: 'GyroStatus' = self._fog_status.get_value()

        if fog_stat is not None and fog_stat.connected:
            if not fog_stat.temp_good:
                stat.summary(DiagnosticStatus.ERROR, "Overheating")
            elif not fog_stat.vsupply_good:
                stat.summary(DiagnosticStatus.ERROR, "Bad Voltage")
            elif not fog_stat.sldcurrent_good:
                stat.summary(DiagnosticStatus.ERROR, "Bad SLDCurrent")
            elif not fog_stat.diagsignal_good:
                stat.summary(DiagnosticStatus.ERROR, "Bad Diag Signal")
            elif not fog_stat.temp_within_cal and fog_stat.temp_good:
                stat.summary(DiagnosticStatus.WARN, "Temperature Out of Cal")
            else:
                stat.summary(DiagnosticStatus.OK, "Connected")    
            
            stat.add("Temperature", str(fog_stat.temperature))
            stat.add("Voltage", str(fog_stat.vsupply))
            stat.add("SLDCurrent", str(fog_stat.sldcurrent))
            stat.add("Diagnostic Signal", str(fog_stat.diagsignal))
        else:
            stat.summary(DiagnosticStatus.ERROR, "Not Connected")
        
        return stat
        

def main():
    hostname = socket.gethostname()
    rclpy.init()
    node = rclpy.create_node("sensor_monitor")
    node.declare_parameter('diag_thresholds_file', rclpy.Parameter.Type.STRING)
    node.declare_parameter('robot', rclpy.Parameter.Type.STRING)
    current_robot = node.get_parameter('robot').value
    with open(node.get_parameter('diag_thresholds_file').value, 'r') as stream:
        thresholds_file = yaml.safe_load(stream)
    message_lifetime = float(thresholds_file["ros_message_lifetime"])
    max_nominal_depth = float(thresholds_file["sensor_monitor_thresholds"]["depth_max_normal"])

    updater = Updater(node)
    updater.setHardwareID(hostname)

    if current_robot == "talos":
        updater.add(DVLSensorTask(node, message_lifetime))
    updater.add(IMUSensorTask(node, message_lifetime))
    updater.add(DepthSensorTask(node, message_lifetime, max_nominal_depth))
    updater.add(FOGSensorTask(node, message_lifetime))

    updater.force_update()

    rclpy.spin(node, None)

if __name__ == '__main__':
    main()