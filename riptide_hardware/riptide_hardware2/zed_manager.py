#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import subprocess
import os
import signal

class ZedManager(Node):
    def __init__(self):
        super().__init__('ZedManager')
        # Create services: sleep_ffc and sleep_dfc; true = sleep (SIGSTOP), false = wake (SIGCONT)
        self.srv_ffc = self.create_service(SetBool, 'sleep_ffc', self.sleep_ffc_callback)
        self.srv_dfc = self.create_service(SetBool, 'sleep_dfc', self.sleep_dfc_callback)
        
        self.get_logger().info("ZedManager node started. Setting default process states...")
        # Set defaults: ffc_container awake and dfc_container asleep
        self.set_default_state('ffc_container', sleep=False)
        self.set_default_state('dfc_container', sleep=True)

    def set_default_state(self, identifier: str, sleep: bool):
        # Set process state by identifier; true => SIGSTOP, false => SIGCONT
        try:
            pid = self.find_process(identifier)
            if pid is not None:
                if sleep:
                    os.kill(pid, signal.SIGSTOP)
                    self.get_logger().info(f"Default: Process {pid} ({identifier}) set to sleep (SIGSTOP).")
                else:
                    os.kill(pid, signal.SIGCONT)
                    self.get_logger().info(f"Default: Process {pid} ({identifier}) set to wake (SIGCONT).")
            else:
                self.get_logger().warn(f"Default: No process found with identifier '{identifier}'.")
        except Exception as e:
            self.get_logger().error(f"Default: Error setting state for {identifier}: {e}")

    def find_process(self, identifier: str):
        # Return the first PID that matches identifier using pgrep; return None if not found
        try:
            pid_output = subprocess.check_output(['pgrep', '-f', identifier])
            pid_list = pid_output.decode().strip().split()
            if pid_list:
                return int(pid_list[0])
            return None
        except subprocess.CalledProcessError:
            return None

    def control_process(self, identifier: str, sleep: bool):
        # Control process state: true => SIGSTOP, false => SIGCONT
        pid = self.find_process(identifier)
        if pid is None:
            return (False, f"No process found matching '{identifier}'.")
        try:
            if sleep:
                os.kill(pid, signal.SIGSTOP)
                return (True, f"Process {pid} ({identifier}) suspended (SIGSTOP).")
            else:
                os.kill(pid, signal.SIGCONT)
                return (True, f"Process {pid} ({identifier}) resumed (SIGCONT).")
        except Exception as e:
            return (False, f"Failed to control process {pid} ({identifier}): {e}")

    def sleep_ffc_callback(self, request, response):
        # For ffc_container: true => sleep, false => wake
        success, message = self.control_process('ffc_container', sleep=request.data)
        response.success = success
        response.message = message
        self.get_logger().info(f"[sleep_ffc]: Request {request.data} - {message}")
        return response

    def sleep_dfc_callback(self, request, response):
        # For dfc_container: true => sleep, false => wake
        success, message = self.control_process('dfc_container', sleep=request.data)
        response.success = success
        response.message = message
        self.get_logger().info(f"[sleep_dfc]: Request {request.data} - {message}")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ZedManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("ZedManager node shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
