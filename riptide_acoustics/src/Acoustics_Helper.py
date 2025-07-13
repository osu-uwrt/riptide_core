#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from riptide_msgs2.msg import DshotRPMFeedback, DshotPartialTelemetry
from geometry_msgs.msg import Vector3Stamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from time import sleep
import os
import datetime
from rclpy.time import Time


#the purpose of this node is to concatinate all thruster data into a single message that includes the latest rpm from each thruster

#TODO figure out real topic names

ACOUSTICS_PINGER_LOCATION = [3.0, 2.0, -1.0, 0.0, 0.0, 0.0]
BASE_FRAME = "map"
TARGET_FRAME_STARTBOARD = "talos/acoustics_starboard_link"
TARGET_FRAME_PORT = "talos/acoustics_port_link"
DATA_FOLDER_NAME = "acoustics_data"


class Acoustics_Helper(Node):

    def __init__(self):
        #init super node
        super().__init__("Acoustics_Helpers")

        self.acoustics_sub = self.create_subscription(Vector3Stamped, "/talos/acoustics/delta_t", self.acoustics_cb, qos_profile_sensor_data) 


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        try:
            #ensure there is a data file
            os.mkdir(DATA_FOLDER_NAME)
        except:
            self.get_logger().info('Created Data Folder')

        #open the log file
        now = datetime.datetime.now()
        # Format the date and time as a string
        formatted_date_time = now.strftime("%Y-%m-%d %H:%M:%S")
        filename = DATA_FOLDER_NAME + "/data" + formatted_date_time + ".csv"
        self.data_file = open(filename, "w")

        #write the file header
        headers = "number,time,delta_t,pos x 1,pos y 1,pos z 1,or w 1,or x 1,or y 1, or z 1,pos x 2,pos y 2,pos z 2,or w 2,or x 2,or y 2, or z 2\n"
        self.data_file.write(headers)

        self.points_written = 0

     

               
    async def acoustics_cb(self, msg):
        #wait to ensure transforms are available
        try:

            delta_t_time = msg.header.stamp

            starboard_transform = await self.tf_buffer.lookup_transform_async(
                BASE_FRAME,
                TARGET_FRAME_STARTBOARD,
                delta_t_time)
            
            port_transform = await self.tf_buffer.lookup_transform_async(
                BASE_FRAME,
                TARGET_FRAME_PORT,
                delta_t_time)


            msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 10e-9
            delta_t = msg.vector.x

            #formulate the output line
            self.points_written = self.points_written + 1

            output_line = str(self.points_written) + "," + str(msg_time) + "," + str(delta_t) + "," + \
                str(starboard_transform.transform.translation.x) + "," + str(starboard_transform.transform.translation.y) + "," + str(starboard_transform.transform.translation.z) + "," + \
                str(starboard_transform.transform.rotation.w) + "," + str(starboard_transform.transform.rotation.x) + "," + str(starboard_transform.transform.rotation.y) + "," + str(starboard_transform.transform.rotation.z) + "," + \
                str(port_transform.transform.translation.x) + "," + str(port_transform.transform.translation.y) + "," + str(port_transform.transform.translation.z) + "," + \
                str(port_transform.transform.rotation.w) + "," + str(port_transform.transform.rotation.x) + "," + str(port_transform.transform.rotation.y) + "," + str(port_transform.transform.rotation.z) + "\n"
                
            #write to csv
            self.data_file.write(output_line)

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform to pinger frame: {ex}')
            return





def main(args=None):

    rclpy.init(args=args)
    node = Acoustics_Helper()
    rclpy.spin(node)
    node.data_file.close()


if __name__ == "__main__":
    main()