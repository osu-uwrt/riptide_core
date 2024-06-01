import gpiod
import rclpy
import time
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from riptide_msgs2.msg import ElectricalCommand

class ImuPowerCycleNode(Node):
    def __init__(self):
        super().__init__('imu_power_cycle')
        self.req = gpiod.line_request()
        self.req.consumer = "imu_power_cycle"
        self.req.request_type=gpiod.line_request.DIRECTION_OUTPUT

        # Get IO Line
        self.gpiochip = gpiod.chip('gpiochip0')
        self.resetline = self.gpiochip.get_line(96)  # Get GPIO 17 (Pin PP.04)
        self.resetline.request(self.req, default_val=1)
        self.nextPowerupTime = time.time()

        # Create the subscription
        self.sub = self.create_subscription(ElectricalCommand, "command/electrical", self.commandCb, qos_profile_system_default)
        self.resetTimer = self.create_timer(1.0, self.resetTimerPoll)

    def resetTimerPoll(self):
        if time.time() < self.nextPowerupTime:
            self.resetline.set_value(0)
        else:
            self.resetline.set_value(1)

    def commandCb(self, msg: 'ElectricalCommand'):
        if msg.command == ElectricalCommand.CYCLE_IMU:
            self.get_logger().info("Power Cycling IMU... Driving Low for 10 seconds")
            self.nextPowerupTime = time.time() + 10


def main(args=None):
    rclpy.init(args=args)
    node = ImuPowerCycleNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
