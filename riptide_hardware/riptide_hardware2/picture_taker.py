#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, SetBool
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime


class ImageCaptureNode(Node):
    def __init__(self):
        super().__init__('image_capture_node')
        
        # Parameters
        self.declare_parameter('robot_namespace', 'talos')
        self.declare_parameter('camera_name', 'ffc')
        self.declare_parameter('save_directory', '/home/ros/cal_images')
        self.declare_parameter('save_stereo', False)
        self.declare_parameter('save_split', True)
        
        # Get parameter values
        robot_namespace = self.get_parameter('robot_namespace').get_parameter_value().string_value
        camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        self.save_directory = self.get_parameter('save_directory').get_parameter_value().string_value
        self.save_stereo = self.get_parameter('save_stereo').get_parameter_value().bool_value
        self.save_split = self.get_parameter('save_split').get_parameter_value().bool_value
        
        # Build the full image topic path
        self.image_topic = f"/{robot_namespace}/{camera_name}/zed_node/stereo_raw/image_raw_color"
        
        # Create save directory if it doesn't exist
        os.makedirs(self.save_directory, exist_ok=True)
        
        # Create subdirectories for split images if needed
        if self.save_split:
            os.makedirs(os.path.join(self.save_directory, 'left'), exist_ok=True)
            os.makedirs(os.path.join(self.save_directory, 'right'), exist_ok=True)
        
        # Initialize CV bridge for image conversion
        self.bridge = CvBridge()
        
        # Store the latest image and subscription state
        self.latest_image = None
        self.subscription_enabled = True
        self.image_subscriber = None
        
        # Log the topic we're subscribing to
        self.get_logger().info(f"Image topic: {self.image_topic}")
        
        # Create initial subscriber for image topic
        self.create_image_subscriber()
        
        # Create service for capturing images
        self.capture_service = self.create_service(
            Trigger,
            'capture_image',
            self.capture_image_callback
        )
        
        # Create service for enabling/disabling subscription
        self.enable_subscription_service = self.create_service(
            SetBool,
            'enable_subscription',
            self.enable_subscription_callback
        )
        
    def create_image_subscriber(self):
        """Create the image subscriber"""
        if self.image_subscriber is None:
            self.image_subscriber = self.create_subscription(
                Image,
                self.image_topic,
                self.image_callback,
                10
            )
            self.get_logger().info(f"Subscribed to image topic: {self.image_topic}")
    
    def destroy_image_subscriber(self):
        """Destroy the image subscriber"""
        if self.image_subscriber is not None:
            self.destroy_subscription(self.image_subscriber)
            self.image_subscriber = None
            self.get_logger().info("Unsubscribed from image topic")

    def image_callback(self, msg):
        try:
            # Convert ROS image message to OpenCV format
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {str(e)}")

    def capture_image_callback(self, request, response):
        if self.latest_image is None:
            response.success = False
            response.message = "No image available"
            return response
        
        try:
            # Generate filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
            
            saved_files = []
            
            # Save stereo image if enabled
            if self.save_stereo:
                stereo_filename = f"captured_image_{camera_name}_{timestamp}.png"
                stereo_filepath = os.path.join(self.save_directory, stereo_filename)
                success = cv2.imwrite(stereo_filepath, self.latest_image, [cv2.IMWRITE_PNG_COMPRESSION, 0])
                
                if success:
                    saved_files.append(f"stereo: {stereo_filename}")
                    self.get_logger().info(f"Stereo image saved: {stereo_filepath}")
                else:
                    self.get_logger().error("Failed to save stereo image")
            
            # Save split left/right images if enabled
            if self.save_split:
                height, width = self.latest_image.shape[:2]
                mid_width = width // 2
                
                # Split the image
                left_image = self.latest_image[:, :mid_width]
                right_image = self.latest_image[:, mid_width:]
                
                # Save left image
                left_filename = f"captured_image_{camera_name}_{timestamp}.png"
                left_filepath = os.path.join(self.save_directory, 'left', left_filename)
                left_success = cv2.imwrite(left_filepath, left_image, [cv2.IMWRITE_PNG_COMPRESSION, 0])
                
                # Save right image
                right_filename = f"captured_image_{camera_name}_{timestamp}.png"
                right_filepath = os.path.join(self.save_directory, 'right', right_filename)
                right_success = cv2.imwrite(right_filepath, right_image, [cv2.IMWRITE_PNG_COMPRESSION, 0])
                
                if left_success:
                    saved_files.append(f"left: {left_filename}")
                    self.get_logger().info(f"Left image saved: {left_filepath}")
                else:
                    self.get_logger().error("Failed to save left image")
                    
                if right_success:
                    saved_files.append(f"right: {right_filename}")
                    self.get_logger().info(f"Right image saved: {right_filepath}")
                else:
                    self.get_logger().error("Failed to save right image")
            
            if saved_files:
                response.success = True
                response.message = f"Images saved - {', '.join(saved_files)}"
            else:
                response.success = False
                response.message = "No images were saved (check save_stereo and save_split parameters)"
                
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
            self.get_logger().error(f"Error capturing image: {str(e)}")
        
        return response
    
    def enable_subscription_callback(self, request, response):
        """Handle enable/disable subscription service calls"""
        try:
            if request.data:  # Enable subscription
                if not self.subscription_enabled:
                    self.create_image_subscriber()
                    self.subscription_enabled = True
                    response.success = True
                    response.message = "Image subscription enabled"
                else:
                    response.success = True
                    response.message = "Image subscription already enabled"
            else:  # Disable subscription
                if self.subscription_enabled:
                    self.destroy_image_subscriber()
                    self.subscription_enabled = False
                    response.success = True
                    response.message = "Image subscription disabled"
                else:
                    response.success = True
                    response.message = "Image subscription already disabled"
                    
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
            self.get_logger().error(f"Error toggling subscription: {str(e)}")
        
        return response


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ImageCaptureNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()