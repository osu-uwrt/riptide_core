#!/usr/bin/env python3
import rclpy
import rclpy.callback_groups
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
from std_msgs.msg import Empty

from time import sleep
from rclpy.executors import MultiThreadedExecutor

class ActuatorTranslator(Node):
    FEEDBACK_PERIOD = 0.2
    
    def __init__(self):
        super().__init__('ActuatorTranslator')
        
        self.torpedo_srv = self.create_service(Trigger, 'command/actuator/torpedo', self.torpedo_cb,)
        self.dropper_srv = self.create_service(Trigger, 'command/actuator/dropper', self.dropper_cb)
        self.notify_reload_srv = self.create_service(Trigger, 'command/actuator/notify_reload', self.notify_reload_cb)
        self.arm_srv = self.create_service(SetBool, 'command/actuator/arm', self.arm_cb)
        
        self.cmd_feedback = self.create_subscription(Bool, "state/actuator/cmd_status", self.feedback_cb, qos_profile_system_default, callback_group=rclpy.callback_groups.ReentrantCallbackGroup())
        self.feedback = False
        self.await_data = True
        
        self.torpedo_pub = self.create_publisher(Empty, "command/actuator/torpedo", qos_profile_system_default)
        self.dropper_pub = self.create_publisher(Empty, "command/actuator/dropper", qos_profile_system_default)
        self.notify_reload_pub = self.create_publisher(Empty, "command/actuator/notify_reload", qos_profile_system_default)
        self.arm_pub = self.create_publisher(Bool, "command/actuator/arm", qos_profile_system_default)
        
        self.rate_sleep = self.create_rate(1 / self.FEEDBACK_PERIOD, self.get_clock())
    
    def pub_empty_with_feedback(self, publisher):
        pub_msg = Empty()
        publisher.publish(pub_msg)
        
        # sleep(self.FEEDBACK_PERIOD)
        # self.rate_sleep.sleep()
        
        self.await_data = True
        while (self.await_data):
            sleep(self.FEEDBACK_PERIOD)
        
        return self.feedback
    
    def torpedo_cb(self, request, response):
        response.success = self.pub_empty_with_feedback(self.torpedo_pub)
        return response

    def dropper_cb(self, request, response):
        response.success = self.pub_empty_with_feedback(self.dropper_pub)
        return response
        
    def notify_reload_cb(self, request, response):
        response.success = self.pub_empty_with_feedback(self.notify_reload_pub)
        return response
        
    def arm_cb(self, request, response):
        arm_msg = Bool()
        arm_msg.data = request.data
        self.arm_pub.publish(arm_msg)
        
        # sleep(self.FEEDBACK_PERIOD)
        # self.rate_sleep.sleep()
        
        self.await_data = True
        while (self.await_data):
            sleep(self.FEEDBACK_PERIOD)
        
        response.success = self.feedback
        return response
        
    def feedback_cb(self, msg : Bool):
        self.feedback = msg.data
        self.await_data = False

def main(args=None):
    rclpy.init(args=args)
    node = ActuatorTranslator()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    
    try:
        # rclpy.spin(node)
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("ActuatorTranslator node shutting down.")

if __name__ == '__main__':
    main()