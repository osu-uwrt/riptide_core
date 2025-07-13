#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import subprocess
import os
import signal
import time
from sensor_msgs.msg import CameraInfo

class ZedManager(Node):
    def __init__(self):
        super().__init__('ZedManager')
        # Create services: sleep_ffc and sleep_dfc; true = sleep (SIGSTOP), false = wake (SIGCONT)
        self.srv_ffc = self.create_service(SetBool, 'sleep_ffc', self.sleep_ffc_callback)
        self.srv_dfc = self.create_service(SetBool, 'sleep_dfc', self.sleep_dfc_callback)
        
        # Joint service to toggle between cameras (use_dfc: true = wake DFC + sleep FFC, false = wake FFC + sleep DFC)
        self.srv_use_dfc = self.create_service(SetBool, 'use_dfc', self.use_dfc_callback)
        
        self.get_logger().info("ZedManager node started. Setting default process states...")
        
        # Declare default topic parameters
        self.declare_parameter('ffc_ready_topic', '/talos/ffc/zed_node/rgb/camera_info')
        self.declare_parameter('dfc_ready_topic', '/talos/dfc/zed_node/rgb/camera_info')
        self.declare_parameter('auto_sleep_ffc', False)  # Whether to auto sleep dfc during init
        self.declare_parameter('auto_sleep_dfc', False) # Whether to auto sleep ffc during init
        
        # Set defaults: ffc_container awake and dfc_container with topic checking auto sleep
        self.init_process('ffc_container', sleep=False, check_topic=self.get_parameter('auto_sleep_ffc').value)
        self.init_process('dfc_container', sleep=False, check_topic=self.get_parameter('auto_sleep_dfc').value)
    
    # Generic callback for when a process is ready during initialization
    def process_ready_callback(self, msg, identifier):
        time.sleep(2.0) # Little delay to make sure everything is up
        self.get_logger().info(f"{identifier} is now ready, putting it to sleep")
        
        # Sleep the process
        pid = self.find_process(identifier)
        if pid is not None:
            os.kill(pid, signal.SIGSTOP)
            self.get_logger().info(f"Process {pid} ({identifier}) suspended (SIGSTOP).")
        else:
            self.get_logger().warn(f"No process found with identifier '{identifier}'.")
        
        # Unsubscribe from the topic since we only need to check once
        if hasattr(self, f"{identifier.split('_')[0]}_subscription"):
            subscription = getattr(self, f"{identifier.split('_')[0]}_subscription")
            self.destroy_subscription(subscription)
            self.get_logger().debug(f"Unsubscribed from {identifier} ready topic")
    
    # Initialize a process - only called during node startup
    def init_process(self, identifier: str, sleep: bool, check_topic: bool = False):
        try:
            pid = self.find_process(identifier)
            if pid is not None:
                if sleep:
                    # Sleep immediately
                    os.kill(pid, signal.SIGSTOP)
                    self.get_logger().info(f"Init: Process {pid} ({identifier}) set to sleep (SIGSTOP).")
                else:
                    # Wake the process
                    os.kill(pid, signal.SIGCONT)
                    
                    if check_topic:
                        # Get the topic name based on the process identifier
                        process_type = identifier.split('_')[0]  # 'dfc' or 'ffc'
                        topic = self.get_parameter(f'{process_type}_ready_topic').value
                        self.get_logger().info(f"Init: Process {pid} ({identifier}) set to wake temporarily. Monitoring topic {topic}")
                        
                        # Create a subscription with a callback for this process
                        subscription = self.create_subscription(
                            CameraInfo,
                            topic,
                            lambda msg: self.process_ready_callback(msg, identifier),
                            10)
                        
                        # Store the subscription with a name based on the process type
                        setattr(self, f"{process_type}_subscription", subscription)
                    else:
                        self.get_logger().info(f"Init: Process {pid} ({identifier}) set to wake (SIGCONT).")
            else:
                self.get_logger().warn(f"Init: No process found with identifier '{identifier}'.")
        except Exception as e:
            self.get_logger().error(f"Init: Error setting state for {identifier}: {e}")
    
    # Return the first PID that matches identifier using pgrep; return None if not found
    def find_process(self, identifier: str):
        try:
            pid_output = subprocess.check_output(['pgrep', '-f', identifier])
            pid_list = pid_output.decode().strip().split()
            if pid_list:
                return int(pid_list[0])
            return None
        except subprocess.CalledProcessError:
            return None
    
    # Control process state: true => SIGSTOP, false => SIGCONT
    def control_process(self, identifier: str, sleep: bool):
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
    
    # For ffc_container: true => sleep, false => wake
    def sleep_ffc_callback(self, request, response):
        success, message = self.control_process('ffc_container', sleep=request.data)
        response.success = success
        response.message = message
        self.get_logger().info(f"[sleep_ffc]: Request {request.data} - {message}")
        return response
    
    # For dfc_container: true => sleep, false => wake
    def sleep_dfc_callback(self, request, response):
        success, message = self.control_process('dfc_container', sleep=request.data)
        response.success = success
        response.message = message
        self.get_logger().info(f"[sleep_dfc]: Request {request.data} - {message}")
        return response
    
    # New service to explicitly use DFC or FFC
    # request.data=True: wake DFC and sleep FFC
    # request.data=False: wake FFC and sleep DFC
    def use_dfc_callback(self, request, response):
        # Check if both processes exist
        ffc_pid = self.find_process('ffc_container')
        dfc_pid = self.find_process('dfc_container')
        
        if ffc_pid is None or dfc_pid is None:
            response.success = False
            response.message = "Switch failed: Unable to find one or both camera processes"
            self.get_logger().warn(response.message)
            return response
        
        try:
            if request.data:  # Use DFC
                # Wake DFC, sleep FFC
                dfc_success, dfc_msg = self.control_process('dfc_container', sleep=False)
                ffc_success, ffc_msg = self.control_process('ffc_container', sleep=True)
                action_description = "Use DFC (wake DFC, sleep FFC)"
            else:  # Use FFC
                # Wake FFC, sleep DFC
                ffc_success, ffc_msg = self.control_process('ffc_container', sleep=False)
                dfc_success, dfc_msg = self.control_process('dfc_container', sleep=True)
                action_description = "Use FFC (wake FFC, sleep DFC)"
            
            # Set response
            success = ffc_success and dfc_success
            message = f"{action_description}: FFC: {ffc_msg}, DFC: {dfc_msg}"
            response.success = success
            response.message = message
            self.get_logger().info(f"[use_dfc]: {request.data} - {message}")
            
        except Exception as e:
            response.success = False
            response.message = f"Switch failed: {str(e)}"
            self.get_logger().error(response.message)
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ZedManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("ZedManager node shutting down.")

if __name__ == '__main__':
    main()