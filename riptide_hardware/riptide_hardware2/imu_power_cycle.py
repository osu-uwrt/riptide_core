#!/usr/bin/env python3

import Jetson.GPIO as GPIO

import rclpy
import time
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from riptide_msgs2.msg import ElectricalCommand

class ImuPowerCycleNode(Node):
    POWER_CYCLE_PIN = 21

    def __init__(self):
        super().__init__('imu_power_cycle')

        # Init GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.POWER_CYCLE_PIN, GPIO.OUT, initial=GPIO.HIGH)

        # Setup subscription and timer
        self.sub = self.create_subscription(ElectricalCommand, "command/electrical", self.commandCb, qos_profile_system_default)
        self.nextPowerupTime = time.time()
        self.resetTimer = self.create_timer(1.0, self.resetTimerPoll)

    def resetTimerPoll(self):
        if time.time() < self.nextPowerupTime:
            GPIO.output(self.POWER_CYCLE_PIN, GPIO.LOW)
        else:
            GPIO.output(self.POWER_CYCLE_PIN, GPIO.HIGH)

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
