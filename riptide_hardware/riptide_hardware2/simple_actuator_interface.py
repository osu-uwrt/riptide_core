import rclpy
import time
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data

from riptide_msgs2.msg import ActuatorStatus

from std_msgs.msg import Empty, Bool
from std_srvs.srv import Trigger, SetBool

from riptide_hardware2.common import ExpiringMessage

import typing
if typing.TYPE_CHECKING:
    from rclpy.client import Client

class SimpleActuatorInterface(Node):
    def __init__(self):
        super().__init__('simple_actuator_interface')

        self.armClient = self.create_client(SetBool, 'command/actuator/arm')
        self.dropperClient = self.create_client(Trigger, 'command/actuator/dropper')
        self.torpedoClient = self.create_client(Trigger, 'command/actuator/torpedo')

        self.torpedoReqSub = self.create_subscription(Empty, "command/simple_torpedo_fire", self.torpedoFireCb, qos_profile_system_default)
        self.dropperReqSub = self.create_subscription(Empty, "command/simple_dropper_fire", self.dropperFireCb, qos_profile_system_default)
        self.statusSub = self.create_subscription(ActuatorStatus, "state/actuator/status", self.actuatorStatus, qos_profile_sensor_data)
        self.busySub = self.create_subscription(Bool, "state/actuator/busy", self.actuatorBusy, qos_profile_sensor_data)

        self.torpedoFireReq = False
        self.dropperFireReq = False
        self.isArmed = ExpiringMessage(self.get_clock(), 3.0)
        self.isBusy = ExpiringMessage(self.get_clock(), 3.0)

    def torpedoFireCb(self, msg):
        self.torpedoFireReq = True

    def dropperFireCb(self, msg):
        self.dropperFireReq = True

    def actuatorStatus(self, msg: 'ActuatorStatus'):
        self.isArmed.update_value(msg.actuators_armed)

    def actuatorBusy(self, msg: 'Bool'):
        self.isBusy.update_value(msg.data)

def fireActuator(node: 'SimpleActuatorInterface', targetClient: 'Client'):
    # Make sure we can get that we're armed
    isArmed = node.isArmed.get_value()
    if isArmed is None:
        node.get_logger().error("No status message received")
        return

    # Make sure we're not busy
    isBusy = node.isBusy.get_value()
    if isBusy is None:
        node.get_logger().error("No busy message received")
        return
    if isBusy:
        node.get_logger().error("Actuator is already busy")
        return

    # Make sure target client is present
    if not targetClient.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("Actuator service not available")
        return

    # Arm if needed
    if not isArmed:
        if not node.armClient.wait_for_service(timeout_sec=5.0):
            node.get_logger().error("Arm service not available")
            return

        # Send arm request
        arm_req = SetBool.Request()
        arm_req.data = True
        future = node.armClient.call_async(arm_req)
        rclpy.spin_until_future_complete(node, future)
        result: 'SetBool.Response' = future.result()

        if not result.success:
            node.get_logger().error("Failed to arm: " + result.message)
            return

        # Wait until armed (or we timeout)
        start_time = time.time()
        while time.time() - start_time < 5.0:
            rclpy.spin_once(node)
            isArmed = node.isArmed.get_value()
            if isArmed is True:
                break

        if isArmed is not True:
            node.get_logger().error("Actuators did not arm within timeout")
            return


    # Send the fire request
    fire_req = Trigger.Request()
    future = targetClient.call_async(fire_req)
    rclpy.spin_until_future_complete(node, future)
    result: 'Trigger.Response' = future.result()
    if not result.success:
        node.get_logger().error("Failed to fire actuator: " + result.message)
        return

    # Wait for busy to clear
    # First delay for a bit for the busy flag to set (shouldn't take over a second)
    start_time = time.time()
    while time.time() - start_time < 1:
        rclpy.spin_once(node)

    # Then wait for isBusy to be clear
    node.isBusy.force_expire()
    start_time = time.time()
    while time.time() - start_time < 5:
        rclpy.spin_once(node)
        isBusy = node.isBusy.get_value()
        if isBusy is False:
            break

    if isBusy is None:
        node.get_logger().error("Busy flag did not publish within timeout")
        return

    if isBusy is not False:
        node.get_logger().error("Torpedo still busy within timeout")
        return

    # Finally disarm
    arm_req = SetBool.Request()
    arm_req.data = False
    future = node.armClient.call_async(arm_req)
    rclpy.spin_until_future_complete(node, future)
    result: 'SetBool.Response' = future.result()

    if not result.success:
        node.get_logger().error("Failed to disarm after firing: " + result.message)
        return

    node.get_logger().info("Successfully Fired Actuator")


def main(args=None):
    rclpy.init(args=args)
    node = SimpleActuatorInterface()
    while True:
        rclpy.spin_once(node)
        if node.dropperFireReq:
            fireActuator(node, node.dropperClient)
            node.dropperFireReq = False
        if node.torpedoFireReq:
            fireActuator(node, node.torpedoClient)
            node.torpedoFireReq = False

if __name__ == '__main__':
    main()
