#! /usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from nortek_dvl_msgs.msg import DvlStatus
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from std_msgs.msg import Float32, String
from queue import Queue
import os


DATA_FOLDER_PATH = "/home/hollis/pvt_data"

class PVT_Monitor(Node):
    pvt_queue = Queue()
    ecage_temp_queue = Queue()
    cc_temp_queue = Queue()
    vectornav_queue = Queue()
    event_queue = Queue()

    def __init__(self):
        super().__init__("PVT_monitor")

        self.pvt_sub = self.create_subscription(Float32, "/talos/state/pvt",  self.pvt_cb, qos_profile_system_default)
        self.ecage_temp_sub = self.create_subscription(Float32, "/talos/state/temp/poacboard",  self.ecage_temp_cb, qos_profile_system_default)
        self.pvt_sub = self.create_subscription(Float32, "/talos/state/temp/cameracage",  self.cc_temp_cb, qos_profile_system_default)
        self.vectornac_sub = self.create_subscription(Float32, "/talos/vectornav/pressure_bar", self.vectornav_cb, qos_profile_system_default)
        self.event_sub = self.create_subscription(String, "/talos/event", self.event_cb, qos_profile_system_default)

        self.create_timer(1, self.write_file_cb)
        

    def pvt_cb(self, msg):
        self.pvt_queue.put([msg.data, self.get_current_time_as_double()])

    def ecage_temp_cb(self, msg):
        self.ecage_temp_queue.put([msg.data, self.get_current_time_as_double()]) 

    def cc_temp_cb(self, msg):
        self.cc_temp_queue.put([msg.data, self.get_current_time_as_double()])    
    
    def vectornav_cb(self, msg):
        self.vectornav_queue.put([msg.data, self.get_current_time_as_double()])

    def event_cb(self, msg):
        self.event_queue.put([msg.data, self.get_current_time_as_double()])

    def write_file_cb(self):

        self.get_logger().info("Writing Data")

        with(open(os.path.join(DATA_FOLDER_PATH, "pvt.csv"), "a")) as file:
            while not (self.pvt_queue.empty()):

                data = self.pvt_queue.get()
                file.write(f"{data[0]},{data[1]},")
        file.close()
            
        with(open(os.path.join(DATA_FOLDER_PATH, "ecage.csv"), "a")) as file:
            while not (self.ecage_temp_queue.empty()):

                data = self.ecage_temp_queue.get()
                file.write(f"{data[0]},{data[1]},")

        file.close()

        
        with(open(os.path.join(DATA_FOLDER_PATH, "cc.csv"), "a")) as file:
            while not (self.cc_temp_queue.empty()):

                data = self.cc_temp_queue.get()
                file.write(f"{data[0]},{data[1]},")
        
        file.close()


        with(open(os.path.join(DATA_FOLDER_PATH, "vectornav.csv"), "a")) as file:
                while not (self.vectornav_queue.empty()):

                    data = self.vectornav_queue.get()
                    file.write(f"{data[0]},{data[1]},")

        file.close()

        with(open(os.path.join(DATA_FOLDER_PATH, "events.csv"), "a")) as file:
                while not (self.event_queue.empty()):

                    data = self.event_queue.get()
                    file.write(f"{data[0]},{data[1]},")

        file.close()

            

    def get_current_time_as_double(self):
        return self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1] * 1e-9
    

def main(args = None):
    rclpy.init(args = args)
    pub = PVT_Monitor()
    rclpy.spin(pub)
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Fake DVL Pub was interrupted.")
