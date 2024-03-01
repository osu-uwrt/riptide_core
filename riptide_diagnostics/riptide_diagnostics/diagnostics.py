import rclpy
from rclpy.node import Node

from std_msgs.msg import String

#! /usr/bin/env python3

#
# colcon build --packages-select
# ros2 run riptide_diagnostics diagnostics.py
print("hello world")

class DiagnosticsSubscriber(Node):
    
    def __init__(self):
        super().__init__('diagnostics_subscriber')
        self.subscription = self.create_subscription(
            String, 'topic', self.listener_callback, 10)
        self.subscription # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    diagnostics_subscriber = DiagnosticsSubscriber()

    # Destroy the node explicitly
    diagnostics_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    