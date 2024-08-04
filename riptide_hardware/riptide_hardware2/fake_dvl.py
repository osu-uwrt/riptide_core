#! /usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from nortek_dvl_msgs.msg import DvlStatus
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

DEFAULT_NOISE = 0.005 #m/s

class FakeDvlPub(Node):
    def __init__(self):
        super().__init__("fake_dvl")
        self.robotName = self.get_namespace()[1 : ] #robot name is namespace without leading /
        self.robotName = self.robotName if not self.robotName.endswith('/') else self.robotName[0 : len(self.robotName) - 1] #remove trailing /
        
        twistTopic = "dvl_twist"
        statusTopic = "dvl/status"

        self.declare_parameter("noise", DEFAULT_NOISE)

        self.twistPub = self.create_publisher(TwistWithCovarianceStamped, twistTopic, qos_profile_sensor_data)
        self.statPub = self.create_publisher(DvlStatus, statusTopic, qos_profile_system_default)
        self.timer = self.create_timer(0.125, self.timerCb)
    
    def timerCb(self):
        now = self.get_clock().now().to_msg()
        dvlFrameId = f"{self.robotName}/dvl_link" if len(self.robotName) > 0 else "dvl_link"

        twist = TwistWithCovarianceStamped()
        twist.header.frame_id = dvlFrameId
        twist.header.stamp = now
        twist.twist.covariance[0] = 1.6641001130282898e-06
        twist.twist.covariance[7] = 1.6641001130282898e-06
        twist.twist.covariance[14] = 1.6641001130282898e-06

        noiseStdDev = self.get_parameter("noise").value
        noise = np.random.normal(0, noiseStdDev, 6)
        twist.twist.twist.linear.x = noise[0]
        twist.twist.twist.linear.y = noise[1]
        twist.twist.twist.linear.z = noise[2]
        twist.twist.twist.angular.x = noise[3]
        twist.twist.twist.angular.y = noise[4]
        twist.twist.twist.angular.z = noise[5]

        self.twistPub.publish(twist)

        status = DvlStatus()
        status.header.stamp = now
        status.header.frame_id = dvlFrameId
        status.b1_dist_valid = True
        status.b1_fom_valid = True
        status.b1_vel_valid = True
        status.b2_dist_valid = True
        status.b2_fom_valid = True
        status.b2_vel_valid = True
        status.b3_dist_valid = True
        status.b3_fom_valid = True
        status.b3_vel_valid = True
        status.b4_dist_valid = True
        status.b4_fom_valid = True
        status.b4_vel_valid = True
        status.x_vel_valid = True
        status.x_fom_valid = True
        status.y_vel_valid = True
        status.y_fom_valid = True
        status.z1_vel_valid = True
        status.z1_fom_valid = True
        status.z2_vel_valid = True
        status.z2_fom_valid = True
        self.statPub.publish(status)



def main(args = None):
    rclpy.init(args = args)
    pub = FakeDvlPub()
    rclpy.spin(pub)
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Fake DVL Pub was interrupted.")
