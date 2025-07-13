#!/usr/bin/env python3

import Jetson.GPIO as GPIO

import rclpy
import time
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from riptide_msgs2.msg import ElectricalCommand

class ImuPowerCycleNode(Node):
    def __init__(self):
        super().__init__('imu_power_cycle')
        
        # figure out which pin to use
        self.declare_parameter("pin_id", 0)
        self.pwrCyclePin = self.get_parameter("pin_id").value
        
        self.get_logger().info(f"IMU power cycle node using pin id {self.pwrCyclePin}")

        # Init GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pwrCyclePin, GPIO.OUT, initial=GPIO.HIGH)

        # Setup subscription and timer
        self.sub = self.create_subscription(ElectricalCommand, "command/electrical", self.commandCb, qos_profile_system_default)
        self.nextPowerupTime = time.time()
        self.resetTimer = self.create_timer(1.0, self.resetTimerPoll)

    def resetTimerPoll(self):
        if time.time() < self.nextPowerupTime:
            GPIO.output(self.pwrCyclePin, GPIO.LOW)
        else:
            GPIO.output(self.pwrCyclePin, GPIO.HIGH)

    def commandCb(self, msg: 'ElectricalCommand'):
        if msg.command == ElectricalCommand.CYCLE_IMU:
            self.get_logger().info("Power Cycling IMU... Driving Low for 10 seconds")
            self.nextPowerupTime = time.time() + 10


def main(args=None):
    rclpy.init(args=args)
    node = ImuPowerCycleNode()
    rclpy.spin(node)

    GPIO.cleanup()

if __name__ == '__main__':
    main()
