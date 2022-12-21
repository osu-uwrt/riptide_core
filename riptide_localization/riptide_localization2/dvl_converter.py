#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from tf2_ros import *
import math
import transforms3d as tf3d
import numpy as np

class dvlConverter(Node):
    def __init__(self):
        super().__init__('riptide_localization2')
        self.dvlSub = self.create_subscription(TwistWithCovarianceStamped, "dvl_twist", self.dvlCb, qos_profile_sensor_data)
        self.odomSub = self.create_subscription(Odometry, "odometry/filtered", self.odomCb, qos_profile_sensor_data)

        self.pub = self.create_publisher(TwistWithCovarianceStamped, "dvl/twist", qos_profile_sensor_data)
        self.namespace = self.get_namespace()[1:]
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.odomTwist = None

    def odomCb(self, msg):
        self.odomTwist = msg.twist.twist

    def dvlCb(self, msg):
        if self.odomTwist is None:
            self.get_logger().warning("Odometry not published yet")
            return

        twist = msg.twist.twist
        try:
            # Transform from dvl to base
            d2bTransform = self.tfBuffer.lookup_transform(self.namespace+'/base_link', self.namespace+'/dvl_link', Time()).transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            self.get_logger().info(str(ex))
            return

        d2bRotation = d2bTransform.rotation
        d2bMatrix = tf3d.quaternions.quat2mat([d2bRotation.w, d2bRotation.x, d2bRotation.y, d2bRotation.z])[:3,:3]
        d2bOffset = d2bTransform.translation
        d2bVector = [d2bOffset.x, d2bOffset.y, d2bOffset.z]

        # Compute added velocity from angular velocity
        base_rot = [self.odomTwist.angular.x, self.odomTwist.angular.y, self.odomTwist.angular.z]
        out_vel = np.dot(d2bMatrix, [twist.linear.x, twist.linear.y, twist.linear.z])
        out_vel += np.cross(d2bVector, base_rot)

        # TODO: Rotate covariance
        msg.header.frame_id = self.namespace+"/base_link"
        twist.linear.x, twist.linear.y, twist.linear.z = out_vel
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = dvlConverter()
    rclpy.spin(node)


if __name__ == '__main__':
    main()   