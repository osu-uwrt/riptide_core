#! /usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray

#
# colcon build --packages-select
# ros2 run riptide_diagnostics diagnostics.py
class DiagnosticsSubscriber(Node):
    
    def __init__(self):
        super().__init__('diagnostics_subscriber')
        self.subscription = self.create_subscription(
            DiagnosticArray, 'diagnostics_agg', self.listener_callback, 10)
        self.subscription # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Recieved data: "%s"\n' % msg)

def main(args=None):
    rclpy.init(args=args)

    diagnostics_subscriber = DiagnosticsSubscriber()
    rclpy.spin(diagnostics_subscriber)

    # Destroy the node explicitly
    diagnostics_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    