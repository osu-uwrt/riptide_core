#!/usr/bin/env python3

import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from riptide_msgs2.msg import UInt8Stamped

#
# This node acts as a fake PHY between the two robots allowing software behaviors involving IVC to be tested without actual hardware
#

class FakeIvcNode(Node):
    def __init__(self):
        super().__init__("fake_ivc")
        
        # declare parameters
        self.declare_parameter("host_robot_name", "")
        self.declare_parameter("remote_robot_name", "")
        self.declare_parameter("failure_rate", -1.0)
        
        self.host_name = self.get_parameter("host_robot_name").value
        self.remote_name = self.get_parameter("remote_robot_name").value
        self.fail_rate = self.get_parameter("failure_rate").value
        
        if self.host_name == "" or self.remote_name == "" or self.fail_rate < 0:
            self.get_logger().error(f"Not all parameters specified! host: {self.host_name} remote: {self.remote_name}, fail rate: {self.fail_rate}")
        
        # handling tx from remote
        self.remote_tx_sub = self.create_subscription(UInt8, f"/{self.remote_name}/ivc/tx", self.handle_remote_tx, 10)
        self.remote_tx_success_pub = self.create_publisher(UInt8Stamped, f"/{self.remote_name}/ivc/tx_success", 10)
        self.host_rx_pub = self.create_publisher(UInt8, f"/{self.host_name}/ivc/rx", 10)
        
        #handling tx from host
        self.host_tx_sub = self.create_subscription(UInt8, f"/{self.host_name}/ivc/tx", self.handle_host_tx, 10)
        self.host_tx_success_pub = self.create_publisher(UInt8Stamped, f"/{self.host_name}/ivc/tx_success", 10)
        self.remote_rx_pub = self.create_publisher(UInt8, f"/{self.remote_name}/ivc/rx", 10)
        
        self.get_logger().info("Fake IVC Node started.")
        
    
    def message_fails(self):
        return random.uniform(0, 1) < self.fail_rate
        
        
    def handle_remote_tx(self, msg):
        if not self.message_fails():
            self.host_rx_pub.publish(msg)
            
            success_msg = UInt8Stamped()
            success_msg.header.stamp = self.get_clock().now().to_msg()
            success_msg.data = msg.data
            self.remote_tx_success_pub.publish(success_msg)
    
    
    def handle_host_tx(self, msg):
        if not self.message_fails():
            self.remote_rx_pub.publish(msg)
            
            success_msg = UInt8Stamped()
            success_msg.header.stamp = self.get_clock().now().to_msg()
            success_msg.data = msg.data
            self.host_tx_success_pub.publish(success_msg)
    

def main(args = None):
    rclpy.init(args = args)
    fake_ivc = FakeIvcNode()
    rclpy.spin(fake_ivc)
    rclpy.shutdown()
    
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
