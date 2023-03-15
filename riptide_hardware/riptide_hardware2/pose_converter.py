#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_geometry_msgs.tf2_geometry_msgs as tf2_geometry_msgs
import tf2_ros
import transforms3d as tf3d
import numpy as np


class poseConverter(Node):
    def __init__(self):
        super().__init__('pose_converter')
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, "zed_node/pose_with_covariance", self.poseCb, qos_profile_sensor_data)
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, "vio/pose", qos_profile_sensor_data)
        self.z2bTransformTimer = self.create_timer(1.0, self.transformCb)
        self.namespace = self.get_namespace()[1:]
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

    def transformCb(self):
        try:
            toFrame = self.namespace+'/base_link'
            fromFrame = self.namespace+'/zed2i_base_link'
            self.z2bTransform = self.tfBuffer.lookup_transform(
                toFrame, fromFrame, Time())

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            self.get_logger().warning(f"{type(ex)} encountered while looking up transform from {fromFrame} to {toFrame}: {str(ex)}")
            return

        self.z2bTransformTimer.cancel()

    def poseCb(self, msg):
        if self.z2bTransformTimer.is_canceled():
            transformedMsg = tf2_geometry_msgs.do_transform_pose_with_covariance_stamped(
                msg, self.z2bTransform)

            # Publish pose from base_link to odom
            outMsg = PoseWithCovarianceStamped()
            outMsg.header = transformedMsg.header
            outMsg.header.frame_id = "odom"
            outMsg.pose.pose.position.x = transformedMsg.pose.pose.position.x
            outMsg.pose.pose.position.y = transformedMsg.pose.pose.position.y
            outMsg.pose.pose.position.z = transformedMsg.pose.pose.position.z
            outMsg.pose.covariance = transformedMsg.pose.covariance
            self.pub.publish(outMsg)


def main(args=None):
    rclpy.init(args=args)
    node = poseConverter()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
