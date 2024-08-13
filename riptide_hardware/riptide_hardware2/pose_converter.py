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
from zed_interfaces.srv import SetPose


class poseConverter(Node):
    def __init__(self):
        super().__init__('pose_converter')
        self.z2bTransformTimer = self.create_timer(1.0, self.transformCb)
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, "zed2i/zed_node/pose_with_covariance", self.poseCb, qos_profile_sensor_data)
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, "vio/pose", qos_profile_sensor_data)
        self.client = self.create_client(SetPose, 'zed2i/zed_node/set_pose')
        self.namespace = self.get_namespace()[1:]
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.future = None
        self.isActuallyReady = False

    def transformCb(self):
        try:
            fromFrame = 'odom'
            fromFrame2 = self.namespace+'/base_link'
            toFrame = self.namespace+'/zed2i_base_link'
            toFrame2 = self.namespace+'/zed2i_base_link'
            self.o2zTransform = self.tfBuffer.lookup_transform(
                toFrame, fromFrame, Time())
            self.z2bTransform = self.tfBuffer.lookup_transform(
                toFrame2, fromFrame2, Time())
            self.b2zTransform = self.tfBuffer.lookup_transform(
                fromFrame2, toFrame2, Time())
            # self.z2bTransformTimer.cancel()
            
            if self.client.service_is_ready():
                if not self.isActuallyReady:
                    return
                request = SetPose.Request()
                request.pos = [self.b2zTransform.transform.translation.x, self.b2zTransform.transform.translation.y, self.b2zTransform.transform.translation.z]
                request.orient = tf3d.euler.quat2euler(tf3d.quaternions.qinverse([self.o2zTransform.transform.rotation.w, self.o2zTransform.transform.rotation.x, self.o2zTransform.transform.rotation.y, self.o2zTransform.transform.rotation.z]))
                if self.future is None:
                    self.future = self.client.call_async(request)
                else:
                    if self.future.result() is not None:
                        if self.future.result().success:
                            self.get_logger().info("We done did something")
                            self.z2bTransformTimer.cancel()
                        else:
                            self.get_logger().warning("Your zed is being stupid")
            # self.get_logger().info(f"X: {self.z2bTransform.transform.translation.x} Y: {self.z2bTransform.transform.translation.y} Z: {self.z2bTransform.transform.translation.z}")


        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            self.get_logger().warning(
                f"{type(ex)} encountered while looking up transform from {fromFrame} to {fromFrame2}: {str(ex)}")
            return

    def poseCb(self, msg):
        if not self.isActuallyReady:
            self.isActuallyReady = True
        if self.z2bTransformTimer.is_canceled():
            transformedMsg = tf2_geometry_msgs.do_transform_pose_with_covariance_stamped(
                msg, self.z2bTransform)

        # Publish pose from base_link to odom
            outMsg = PoseWithCovarianceStamped()
            outMsg.header.frame_id = 'odom'
            outMsg.pose.pose.position.x = transformedMsg.pose.pose.position.x
            outMsg.pose.pose.position.y = transformedMsg.pose.pose.position.y
            outMsg.pose.pose.position.z = transformedMsg.pose.pose.position.z
            outMsg.pose.covariance = msg.pose.covariance
            outMsg.header.stamp = msg.header.stamp
            self.pub.publish(outMsg)


def main(args=None):
    rclpy.init(args=args)
    node = poseConverter()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
