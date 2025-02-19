#!/usr/bin/env python3

import os

import rclpy
from rclpy.action import ActionServer
import rclpy.clock
from rclpy.node import Node

from time import sleep

import numpy as np
from math import sqrt
from scipy import stats
import yaml

from riptide_msgs2.action import Depressurize
from std_msgs.msg import Float32
from rclpy.qos import qos_profile_sensor_data

#load these from yaml eventually
TEMP_STD_DEV_TOLERANCE = 2
PRESSURE_STD_DEV_TOLERANCE = .01 #bar
PRESSURIZATION_TOLERANCE = .02 #bar
HULL_VOLUME = 1 #not technically needed but makes me feel good inside
TRUTH_CERTAINTY = .99

PUBLISH_INTERVAL = 1

#this is cursed and I frankly dont care
#the pressurization file needs to be persistant across colcon builds so this file needs to hide somewhere safe
PRESSURIZATION_LOG_LOCATION = "/home/dev/pressurization_log.yaml"

class SampleSet():
    #set of pressure and temp samples

    #camera cage 
    camera_cage_temp = None
    camera_cage_temp_stamp = None

    #ecage
    ecage_temp = None
    ecage_temp_stamp = None

    #pressure
    pressure = None
    pressure_stamp = None

    def __init__(self, start_time):
        #add the start
        self.start_time = start_time

    def add_pressure_sample(self, pressure, time):
        #add a pressure stamp
        if not (self.pressure is None):
            self.pressure.append(pressure)
            self.pressure_stamp.append(time)
        else:
            self.pressure = np.array([pressure])
            self.pressure_stamp = np.array([time])

    def add_ecage_sample(self, temp, time):
        #add a pressure stamp
        if not (self.pressure is None):
            self.pressure.append(temp)
            self.pressure_stamp.append(time)
        else:
            self.pressure = np.array([temp])
            self.pressure_stamp = np.array([time])

    def add_camera_cage_sample(self, temp, time):
        #add a pressure stamp
        if not (self.pressure is None):
            self.pressure.append(temp)
            self.pressure_stamp.append(time)
        else:
            self.pressure = np.array([temp])
            self.pressure_stamp = np.array([time])

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

        pvt_array = None
        
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

            if not (pvt_array is None):
                pvt_array.append(pvt)
            else:
                pvt_array = np.array([pvt])

        #return the average pvt value
        return np.mean(pvt_array)
    
    def get_pvt_std(self):
        #assuming samples are added in order
        
        camera_cage_sample_index = 0
        camera_cage_sample_time = self.camera_cage_temp_stamp[0]
        ecage_cage_sample_index = 0
        ecage_sample_time = self.ecage_temp_stamp[0]

        pvt_array = None
        
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

            if not (pvt_array is None):
                pvt_array.append(pvt)
            else:
                pvt_array = np.array([pvt])

        #return the average pvt value
        return np.std(pvt_array)

    def get_num_samples(self):
        #return the number of pressure samples taken
        return len(self.pressure)


class PressureMonitor(Node):

    #the sample set currently collecting samples
    #discards sample while set to none
    collecting_sample_set = None

    # the current hull pressure
    current_pressure = 1

    #depressurized pvts
    depressurized_pvt = None
    depressurized_pvt_std = None
    depressurized_pvt_samples = None

    #sampling of depressurization status
    sampling_time = None

    def __init__(self):
        super().__init__('pressure_monitor')

        self.depressurization_server = ActionServer(self, Depressurize,'depressurize', self.depressurize_callback)
        self.pressure_sub = self.create_subscription(Float32, "vectornav/pressure_bar", self.update_pressure, qos_profile_sensor_data)
        self.ecage_temp_sub = self.create_subscription(Float32, "state/poac/temp", self.update_ecage_temp, qos_profile_sensor_data)
        self.camera_cage_temp_sub = self.create_subscription(Float32, "state/camera_cage_bb/temp", self.update_camera_cage_temp, qos_profile_sensor_data)

        #create callback to check depressurization status
        self.create_timer(.05, self.check_pressurization_status)

        #load in depressurization file
        self.load_pressurization_file()

    def load_pressurization_file(self):
        # try to open the depressurization log
        try:
            with open(PRESSURIZATION_LOG_LOCATION, "r") as file:
                depressurization_yaml = yaml.safe_load(file)

                #load in parameters
                self.depressurized_pvt = depressurization_yaml["pvt"]
                self.depressurized_pvt_std = depressurization_yaml["pvt_std"]
                self.depressurized_pvt_samples = depressurization_yaml["pvt_samples"]

        except FileNotFoundError as e:
            self.get_logger().warn("Could not find pressurization log! - This could be normal. Did you expect the vehicle to be pressurized?")

        except KeyError as e:
            self.get_logger().warn("The depressurization file was corrupted! This file will be delete and you should check the pressurization status!")

            #bring everything to a known state
            self.depressurized_pvt = None
            self.depressurized_pvt_std = None
            self.depressurized_pvt_samples = None

            self.remove_depressurization_log()
    
    def depressurize_callback(self, goal_handle):
        self.get_logger().info('Starting Depressurization')

        #clear the previous depressurization state if any
        self.depressurized_pvt = None
        self.depressurized_pvt_std = None
        self.depressurized_pvt_samples = None
        
        #get the sampling time
        sample_time = goal_handle.request.sampling_time

        #set the node's sampling time
        self.sampling_time = sample_time

        #add sample set to target
        self.collecting_sample_set = SampleSet(self.get_current_time_as_double())

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
        sampled_final_pvt_std = 0
        sampled_final_pvt_samples = 0
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
                self.collecting_sample_set = SampleSet(self.get_current_time_as_double())

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
                    sampled_final_pvt_std = current_samples.get_pvt_std()
                    sampled_final_pvt_samples = current_samples.get_num_samples()
                    sampled_cleanly = True

                    self.get_logger().info(f"Current Pressure is {sampled_final_pressure}. Samples were taken cleanly!")
                else:
                    self.get_logger().info("Samples were dirty. Retaking samples!")

        #publish result
        result_msg = Depressurize.Result()
        result_msg.success = True
        result_msg.pvt = sampled_final_pvt
        
        #record pvt characteristics
        self.depressurized_pvt = sampled_final_pvt
        self.depressurized_pvt_std = sampled_final_pvt_std
        self.depressurized_pvt_samples = sampled_final_pvt_samples

        #write the depressurization log
        self.write_depressurization_log()

        goal_handle.succeed()

        return result_msg             

    def check_pressurization_status(self):

        if(self.depressurized_pvt is None) or (self.sampling_time is None):
            #depressurization is not active
            return
        
        if(self.collecting_sample_set is None):
            #start a sample collection
            self.collecting_sample_set = SampleSet(self.get_current_time_as_double())

            return

        if(self.collecting_sample_set.start_time + self.sampling_time < self.get_current_time_as_double()):
            #keep collecting and wait

            return
        
        #get the stats on the current sample set
        current_pvt = self.collecting_sample_set.get_pvt()
        current_pvt_std = self.collecting_sample_set.get_pvt_std()
        current_pvt_samples = self.collecting_sample_set.get_num_samples()

        #run a truth test on wether or not the amount of gas in the hull is the same
        if(self.truth_test(self.depressurized_pvt, current_pvt,self.depressurized_pvt_std, current_pvt_std, self.depressurized_pvt_samples, current_pvt_samples) < TRUTH_CERTAINTY):
            #depressurization has been detected

            #probably make a bigger fuss than this
            self.get_logger().error("Depressurization Detected!!!!!!!!!")

            #reset the system
            self.depressurized_pvt = None
            self.depressurized_pvt_std = None
            self.depressurized_pvt_samples = None

            #remove depressurization log
            self.remove_depressurization_log()

            return
        
        #clear samples and start again
        self.collecting_sample_set = None
            
    def truth_test(self, mean1, mean2, std1, std2, samples1, samples2):
        #generic t test returns the probability that the samples are significantly different

        #check to make sure enought samples
        if(samples2 < 2 or samples1 < 1):
            self.get_logger().info("Severe Lack of Samples - take a look at sum")
            return 0

        #using Welch's T-Test - uneven varience and uneven number of samples 
        #ref: https://www.investopedia.com/terms/t/t-test.asp
        t_value = (mean1 - mean2) / sqrt((std1^2 / samples1) + (std2^2 / samples2))
        dof = ((std1^4 / samples1) + (std2^4 / samples2))^2 / ((std1^4 / samples1)^2 / (samples1 - 1) + (std2^4 / samples2)^2 / (samples2 - 1))

        #calculare p-value
        p = stats.t.cdf(t_value, df=dof)

        #convert to two tailed distribution
        p_two_tail = 2 * (1 - abs(p - 0.5) - 0.5) # this was a gemini moment

        return p_two_tail
        
    def write_depressurization_log(self):
        #write the depressurization log

        data = dict()
        data["pvt"] = self.depressurized_pvt
        data["pvt_std"] = self.depressurized_pvt_std
        data["pvt_samples"] = self.depressurized_pvt_samples

        try:
            with open(PRESSURIZATION_LOG_LOCATION, "w") as file:
                #write out to yaml
                yaml.dump(data, file)

        except FileNotFoundError as e:
            self.get_logger().warn("Could not open depressurization log for writing!")


    def remove_depressurization_log(self):
        #delete the depressurization log file so that the robot does expect it to be pressurized on boot
        if os.path.exists(PRESSURIZATION_LOG_LOCATION):
            os.remove(PRESSURIZATION_LOG_LOCATION)
        else:
            self.get_logger().warn("Could not remove depressurization log because it does not exist!") 

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