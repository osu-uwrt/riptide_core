import time

import rclpy
from rclpy.action import ActionServer
import rclpy.clock
from rclpy.node import Node

from time import sleep

import numpy as np

from action_tutorials_interfaces.action import Fibonacci
from riptide_msgs2.action import Depressurize
from std_msgs.msg import Float32
from rclpy.qos import qos_profile_sensor_data

#load these from yaml eventually
TEMP_STD_DEV_TOLERANCE = 2
PRESSURE_STD_DEV_TOLERANCE = .01 #bar
PRESSURIZATION_TOLERANCE = .02 #bar
HULL_VOLUME = 1 #not technically needed but makes me feel good inside

PUBLISH_INTERVAL = 1


class SampleSet():
    #set of pressure and temp samples

    #camera cage 
    camera_cage_temp = np.array()
    camera_cage_temp_stamp = np.array()

    #ecage
    ecage_temp = np.array()
    ecage_temp_stamp = np.array()

    #pressure
    pressure = np.array()
    pressure_stamp = np.array()

    def add_pressure_sample(self, pressure, time):
        #add a pressure stamp
        self.pressure.append(pressure)
        self.pressure_stamp.append(time)

    def add_ecage_sample(self, temp, time):
        #add a pressure stamp
        self.ecage_temp.append(temp)
        self.ecage_temp_stamp.append(time)

    def add_camera_cage_sample(self, temp, time):
        #add a pressure stamp
        self.camera_cage_temp.append(temp)
        self.camera_cage_temp_stamp.append(time)

    def get_average_pressure(self):
        #get average of pressure sample set
        return np.mean(self.pressure)
    
    def get_pressure_std_dev(self):
        #get std  dev of pressure sample set
        return np.std(self.pressure)

    def get_ecage_temp_std_dev(self):
        #get std dev of ecage sample set
        return np.std(self.camera_cage_temp_stamp)

    def get_camera_cage_temp_std_dev(self):
        #get std dev of camera cage sample set
        return np.std(self.ecage_temp)
    
    def get_pvt(self):
        #assuming samples are added in order
        
        camera_cage_sample_index = 0
        camera_cage_sample_time = self.camera_cage_temp_stamp[0]
        ecage_cage_sample_index = 0
        ecage_sample_time = self.ecage_temp_stamp[0]

        pvt_array = np.array()
        
        #for each pressure sample, calculate the average pvt
        for sample, i in enumerate(self.pressure):

            #get the pressure sample stamp
            sample_time = self.pressure_stamp[i]

            #find the closest camera cage sample time
            while(sample_time > camera_cage_sample_time) and (camera_cage_sample_index < len(self.camera_cage_temp_stamp) - 1):
                camera_cage_sample_index += 1
                camera_cage_sample_time = self.camera_cage_temp_stamp[camera_cage_sample_index]

            #find the closest ecage cage sample time
            while(sample_time > ecage_sample_time) and (ecage_cage_sample_index < len(self.ecage_temp_stamp) - 1):
                ecage_cage_sample_index += 1
                ecage_sample_time = self.ecage_temp_stamp[ecage_cage_sample_index]

            average_temp_K = (self.camera_cage_temp[camera_cage_sample_index] + self.ecage_temp[ecage_cage_sample_index]) / 2 + 273.15
            pvt = sample * HULL_VOLUME / (average_temp_K)

            pvt_array.append(pvt)

        #return the average pvt value
        return np.mean(pvt_array)

            



class PressureMonitor(Node):

    #the sample set currently collecting samples
    #discards sample while set to none
    collecting_sample_set = None

    # the current hull pressure
    current_pressure = 1

    def __init__(self):
        super().__init__('pressure_monitor')

        self._action_server = ActionServer(self, Fibonacci,'fibonacci', self.execute_callback)
        self.depressurization_server = ActionServer(self, Depressurize,'depressurize', self.depressurize_callback)
        self.pressure_sub = self.create_subscription(Float32, "state/housing_pressure", self.add_pressure_sample, qos_profile_sensor_data)
        self.ecage_temp_sub = self.create_subscription(Float32, "state/poac/temp", self.update_ecage_temp, qos_profile_sensor_data)
        self.camera_cage_temp_sub = self.create_subscription(Float32, "state/camera_cage_bb/temp", self.update_camera_cage_temp, qos_profile_sensor_data)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result
    
    def depressurize_callback(self, goal_handle):
        self.get_logger().info('Starting Depressurization')
        
        #get the sampling time
        sample_time = goal_handle.request.sampling_time

        #add sample set to target
        self.collecting_sample_set = SampleSet()

        sample_start_time = self.get_current_time_as_double()

        #identify current pressure
        while(sample_start_time + sample_time < self.get_current_time_as_double()):
            sleep(.01)

        #copy the sample set
        initial_samples = self.collecting_sample_set

        #stop sampling
        self.collecting_sample_set = None

        #decide if initial readings were okay
        initial_pressure = 0
        if(initial_samples.get_pressure_std_dev() < PRESSURE_STD_DEV_TOLERANCE and initial_samples.get_ecage_temp_std_dev() < TEMP_STD_DEV_TOLERANCE and initial_samples.get_camera_cage_temp_std_dev() < TEMP_STD_DEV_TOLERANCE):
            initial_pressure = initial_samples.get_average_pressure()
        else:
            self.get_logger().error(f"Cannot proceed with depressurization, the pressure or temperature is too unstable! Pressure Dev: {initial_samples.get_pressure_std_dev()} Ecage Temp Dev: {initial_samples.get_ecage_temp_std_dev()} Camera Cage Dev: {initial_samples.get_camera_cage_temp_std_dev()}")
            goal_handle.abort()
            return Depressurize.Result()
        
        #determine the goal pressure
        target_pressure = initial_pressure - goal_handle.request.net_depressuization
        self.get_logger().info(f"Initial Hull Pressure: {initial_pressure} Target Hull Pressure: {target_pressure}")

        self.get_logger().info("Begin Lowering Pressure!")
        #wait while pressure is lowered
        sampled_final_pressure = initial_pressure
        sampled_final_pvt = 0 
        while(sampled_final_pressure > target_pressure + PRESSURIZATION_TOLERANCE):

            #wait untile pressure dips below target pressure
            last_pub_time = self.get_current_time_as_double()
            while(self.current_pressure < target_pressure):
                if(last_pub_time + PUBLISH_INTERVAL > self.get_current_time_as_double()):
                    #print state
                    self.get_logger.info(f"Continue Lowering Pressure. Current Hull Pressure {self.current_pressure}. Target Pressure: {target_pressure}.")

                    #publish feedback
                    feedback_msg = Depressurize.Feedback()
                    feedback_msg.current_pressure = self.current_pressure
                    goal_handle.publish_feedback()

                    #update the publish time
                    last_pub_time = self.get_current_time_as_double()

            #get a clean pressure sample
            sampled_cleanly = False 

            while(not sampled_cleanly):
                #run sampling to detect if pressurization is stable
                self.get_logger().info("Please wait! Collecting Pressure Sample Data!")

                sleep(1)

                self.get_logger().info("Collection begining!")

                #add sample set to target
                self.collecting_sample_set = SampleSet()

                sample_start_time = self.get_current_time_as_double()

                #identify current pressure
                while(sample_start_time + sample_time < self.get_current_time_as_double()):
                    sleep(.01)
                    
                #copy the sample set
                current_samples = self.collecting_sample_set
                self.collecting_sample_set = None

                #check to see if samples are satisfactory
                if(current_samples.get_pressure_std_dev() < PRESSURE_STD_DEV_TOLERANCE and current_samples.get_ecage_temp_std_dev() < TEMP_STD_DEV_TOLERANCE and current_samples.get_camera_cage_temp_std_dev() < TEMP_STD_DEV_TOLERANCE):
                    sampled_final_pressure = current_samples.get_average_pressure()
                    sampled_final_pvt = current_samples.get_pvt()
                    sampled_cleanly = True

                    self.get_logger().info(f"Current Pressure is {sampled_final_pressure}. Samples were taken cleanly!")
                else:
                    self.get_logger().info("Samples were dirty. Retaking samples!")

        #publish result
        result_msg = Depressurize.Result()
        result_msg.success = True
        result_msg.pvt = sampled_final_pvt

        goal_handle.succeed()

        return result_msg


                    







                
        

        

        

    def update_pressure(self, msg):
        self.current_pressure = msg.data

        if not (self.collecting_sample_set is None):
            #add sample
            self.collecting_sample_set.add_pressure_sample(msg.data, self.get_current_time_as_double())

    def update_camera_cage_temp(self, msg):
        if not (self.collecting_sample_set is None):
            #add sample
            self.collecting_sample_set.add_camera_cage_sample(msg.data, self.get_current_time_as_double())

    def update_ecage_temp(self, msg):
        if not (self.collecting_sample_set is None):
            #add sample
            self.collecting_sample_set.add_camera_cage_sample(msg.data, self.get_current_time_as_double())

    def get_current_time_as_double(self):
        return self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1] * 1e-9




def main(args=None):
    rclpy.init(args=args)

    pressure_monitor = PressureMonitor()

    rclpy.spin(pressure_monitor)


if __name__ == '__main__':
    main()