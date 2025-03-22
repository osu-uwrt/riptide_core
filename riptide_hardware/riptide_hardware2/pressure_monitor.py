#!/usr/bin/env python3

import os
from enum import Enum
import numpy as np
from math import sqrt, log, exp
from scipy import stats
import yaml

from time import sleep
from threading import Lock

import rclpy
from rclpy.action import ActionServer
import rclpy.clock
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

from riptide_msgs2.action import Depressurize
from riptide_msgs2.msg import LedCommand
from std_msgs.msg import Float32

from ament_index_python import get_package_share_directory

#load these from yaml eventually
HULL_VOLUME = 1 #not technically needed but makes me feel good inside
TRUTH_CERTAINTY = -100

PUBLISH_INTERVAL = 1

#this is cursed and I frankly dont care
#the pressurization file needs to be persistant across colcon builds so this file needs to hide somewhere safe
PRESSURIZATION_LOG_LOCATION = "/home/ros/pressurization_log.yaml"
MAX_SAMPLES = 10000

class LedStates(Enum):
    OFF = 0
    QUICK_ANALYZE = 1
    LONG_ANALYZE = 2
    ALL_GOOD = 3
    TEST_FAILED = 4
    KEEP_PUMPING = 5
    REDUCE_PRESSURE = 6
    PUMPING_COMPLETE = 7
    DIRTY_SAMPLES = 8

class SampleSet():
    #set of pressure and temp samples

    #camera cage 
    camera_cage_temp = None
    camera_cage_temp_stamp = None
    camera_cage_index = 0

    #ecage
    ecage_temp = None
    ecage_temp_stamp = None
    ecage_index = 0

    #pressure
    pressure = None
    pressure_stamp = None
    pressure_index = 0

    #thread safe lock
    lock = None

    def __init__(self, start_time):
        #add the start
        self.start_time = start_time

        # create the thread lock            
        self._lock = Lock()


    def add_pressure_sample(self, pressure, time):
        with self._lock:
            #add a pressure stamp
            if (not (self.pressure is None)) and (self.pressure_index < MAX_SAMPLES):
                self.pressure[self.pressure_index] = pressure
                self.pressure_stamp[self.pressure_index] = time
                
                self.pressure_index +=1
                
                return self.pressure_index
            else:
                self.pressure = np.zeros(MAX_SAMPLES)
                self.pressure_stamp = np.zeros(MAX_SAMPLES)
                
                self.pressure_index = 0
                
                self.pressure[self.pressure_index] = pressure
                self.pressure_stamp[self.pressure_index] = time
                
                self.pressure_index +=1
                
                return 0

    def add_ecage_sample(self, temp, time):
        with self._lock:
            #add a pressure stamp
            if (not (self.ecage_temp is None)) and (self.ecage_index < MAX_SAMPLES):
                self.ecage_temp[self.ecage_index] = temp
                self.ecage_temp_stamp[self.ecage_index] = time
                
                self.ecage_index +=1
                
                return self.ecage_index
            else:
                self.ecage_temp = np.zeros(MAX_SAMPLES)
                self.ecage_temp_stamp = np.zeros(MAX_SAMPLES)
                
                self.ecage_index = 0
                
                self.ecage_temp[self.ecage_index] = temp
                self.ecage_temp_stamp[self.ecage_index] = time
                
                self.ecage_index +=1
                
                return 0

    def add_camera_cage_sample(self, temp, time):
        with self._lock:
            #add a pressure stamp
            if (not (self.camera_cage_temp is None)) and (self.camera_cage_index < MAX_SAMPLES):
                self.camera_cage_temp[self.camera_cage_index] = temp
                self.camera_cage_temp_stamp[self.camera_cage_index] = time
                
                self.camera_cage_index +=1
                
                return self.camera_cage_index
            else:
                self.camera_cage_temp = np.zeros(MAX_SAMPLES)
                self.camera_cage_temp_stamp = np.zeros(MAX_SAMPLES)
                
                self.camera_cage_index = 0
                
                self.camera_cage_temp[self.camera_cage_index] = temp
                self.camera_cage_temp_stamp[self.camera_cage_index] = time
                
                self.camera_cage_index +=1
                
                return 0


    def get_average_pressure(self):
        with self._lock:

            if not (self.pressure is None):
                #get average of pressure sample set
                return np.mean(self.pressure[0:self.pressure_index])
            else:
                return -1
    
    def get_pressure_std_dev(self):
        with self._lock:
            #get std  dev of pressure sample set
            if not (self.pressure is None):
                return np.std(self.pressure[0:self.pressure_index])
            else:
                return -1

    def get_ecage_temp_std_dev(self):
        with self._lock:
            #get std dev of ecage sample set
            if not (self.ecage_temp is None):
                return np.std(self.ecage_temp[0:self.ecage_index])
            else:
                return -1

    def get_camera_cage_temp_std_dev(self):
        with self._lock:
            #get std dev of camera cage sample set
            if not (self.camera_cage_temp is None):
                return np.std(self.camera_cage_temp[0:self.camera_cage_index])
            else:
                return -1
        
    def get_pvt(self):
        with self._lock:
            #assuming samples are added in order

            if (self.camera_cage_temp is None) or (self.ecage_temp is None) or (self.pressure is None):
                return -1

            
            camera_cage_sample_index = 0
            camera_cage_sample_time = self.camera_cage_temp_stamp[0]
            ecage_cage_sample_index = 0
            ecage_sample_time = self.ecage_temp_stamp[0]

            pvt_array = np.zeros(MAX_SAMPLES)
            
            #for each pressure sample, calculate the average pvt
            for i, sample in enumerate(self.pressure[0:self.pressure_index]):

                #get the pressure sample stamp
                sample_time = self.pressure_stamp[i]

                #find the closest camera cage sample time
                while(sample_time > camera_cage_sample_time) and (camera_cage_sample_index < self.camera_cage_index - 1):
                    camera_cage_sample_index += 1
                    camera_cage_sample_time = self.camera_cage_temp_stamp[camera_cage_sample_index]

                #find the closest ecage cage sample time
                while(sample_time > ecage_sample_time) and (ecage_cage_sample_index < self.ecage_index - 1):
                    ecage_cage_sample_index += 1
                    ecage_sample_time = self.ecage_temp_stamp[ecage_cage_sample_index]

                average_temp_K = (self.camera_cage_temp[camera_cage_sample_index] + self.ecage_temp[ecage_cage_sample_index]) / 2 + 273.15
                pvt = float(sample * HULL_VOLUME / (average_temp_K))

                pvt_array[i] = pvt


            #return the average pvt value
            return np.mean(pvt_array[1:self.pressure_index])
    
    def get_pvt_std(self):
        with self._lock:
            #assuming samples are added in order

            if (self.camera_cage_temp is None) or (self.ecage_temp is None) or (self.pressure is None):
                return -1
            
            camera_cage_sample_index = 0
            camera_cage_sample_time = self.camera_cage_temp_stamp[0]
            ecage_cage_sample_index = 0
            ecage_sample_time = self.ecage_temp_stamp[0]

            pvt_array = np.zeros(MAX_SAMPLES)
            
            #for each pressure sample, calculate the average pvt
            for i, sample in enumerate(self.pressure[0:self.pressure_index]):

                #get the pressure sample stamp
                sample_time = self.pressure_stamp[i]

                #find the closest camera cage sample time
                while(sample_time > camera_cage_sample_time) and (camera_cage_sample_index < self.camera_cage_index - 1):
                    camera_cage_sample_index += 1
                    camera_cage_sample_time = self.camera_cage_temp_stamp[camera_cage_sample_index]

                #find the closest ecage cage sample time
                while(sample_time > ecage_sample_time) and (ecage_cage_sample_index < self.ecage_index - 1):
                    ecage_cage_sample_index += 1
                    ecage_sample_time = self.ecage_temp_stamp[ecage_cage_sample_index]

                average_temp_K = (self.camera_cage_temp[camera_cage_sample_index] + self.ecage_temp[ecage_cage_sample_index]) / 2 + 273.15
                pvt = float(sample * HULL_VOLUME / (average_temp_K))

                pvt_array[i] = pvt


            #return the average pvt value
            return np.std(pvt_array[1:self.pressure_index])

    def get_num_samples(self):
        with self._lock:
            #return the number of pressure samples taken
            if not (self.pressure is None):
                return self.pressure_index
            else:
                return 0
        
    def is_filled(self):
        with self._lock:
            
            #reutrn if all fields have been filled
            return (self.pressure is not None) and (self.ecage_temp is not None) and (self.camera_cage_temp is not None)


class PressureMonitor(Node):

    #the sample set currently collecting samples
    #discards sample while set to none
    collecting_sample_set = None

    # the current hull pressure
    current_pressure = 1

    # initial pvt
    initial_pvt = None

    #depressurized pvts
    depressurized_pvt = None
    depressurized_pvt_std = None
    depressurized_pvt_samples = None
    pvt_leak_rate_compensation = None #allows for the max pvt to be lowered such that the pvt checker also makes sure the AUV does not leak at a fast rate

    #when the pressurization initially occured
    initial_pressurization_time = None

    #sampling of depressurization status
    sampling_time = None
    
    turnoff_led_timer = None
    
    #current pvt
    current_pvt_w_state = None

    def __init__(self):
        super().__init__('pressure_monitor')

        #create call back groups so that the action servver callback can be blocking
        self.depressurization_routine_group = MutuallyExclusiveCallbackGroup()
        self.general_callback_group = MutuallyExclusiveCallbackGroup()

        self.depressurization_server = ActionServer(self, Depressurize,'depressurize', self.depressurize_callback, callback_group=self.depressurization_routine_group)

        self.pressure_sub = self.create_subscription(Float32, "vectornav/pressure_bar", self.update_pressure, qos_profile_sensor_data, callback_group=self.general_callback_group)
        self.ecage_temp_sub = self.create_subscription(Float32, "state/temp/poacboard", self.update_ecage_temp, qos_profile_sensor_data, callback_group=self.general_callback_group)
        self.camera_cage_temp_sub = self.create_subscription(Float32, "state/temp/cameracage", self.update_camera_cage_temp, qos_profile_sensor_data, callback_group=self.general_callback_group)

        self.led_pub = self.create_publisher(LedCommand, "command/led", qos_profile_system_default)
        self.pressure_pub = self.create_publisher(Float32, "state/pvt", qos_profile_system_default)

        #create callback to check depressurization status
        self.create_timer(.1, self.check_pressurization_status, callback_group=self.general_callback_group)
        
        #create broadcast timer
        self.create_timer(.1, self.broadcast_pvt_state, callback_group=self.general_callback_group)

        #load parameters callback
        self.create_timer(1, self.reload_params, callback_group=self.general_callback_group)

        #load in depressurization file
        self.load_pressurization_file()

        #load in parameters
        self.load_parameters()

    def load_pressurization_file(self):
        # try to open the depressurization log
        try:
            with open(PRESSURIZATION_LOG_LOCATION, "r") as file:
                depressurization_yaml = yaml.safe_load(file)

                #load in parameters
                self.depressurized_pvt = depressurization_yaml["pvt"]
                self.depressurized_pvt_std = depressurization_yaml["pvt_std"]
                self.depressurized_pvt_samples = depressurization_yaml["pvt_samples"]
                self.sampling_time = depressurization_yaml["sampling_time"]
                self.initial_pressurization_time = depressurization_yaml["initial_pressurization_time"]
                self.initial_pvt = depressurization_yaml["initial_pvt"]
                self.pvt_leak_rate_compensation = depressurization_yaml["leak_rate_compensation"]

        except FileNotFoundError as e:
            self.get_logger().warn("Could not find pressurization log! - This could be normal. Did you expect the vehicle to be pressurized?")

        except KeyError as e:
            self.get_logger().warn("The depressurization file was corrupted! This file will be deleted and you should check the pressurization status!")

            #bring everything to a known state
            self.depressurized_pvt = None
            self.depressurized_pvt_std = None
            self.depressurized_pvt_samples = None
            self.pvt_leak_rate_compensation = None

            self.remove_depressurization_log()
            
        except yaml.constructor.ConstructorError as e:
            self.get_logger().warn("Depressurization log is Ahhh Goofy. This file will be delete and you should check the pressurization status!")
            
            #bring everything to a known state
            self.depressurized_pvt = None
            self.depressurized_pvt_std = None
            self.depressurized_pvt_samples = None
            self.pvt_leak_rate_compensation = None

            self.remove_depressurization_log()

    def load_parameters(self,):
        #load in ros2 parameter

        self.declare_parameter("robot", "")
        self.robotName = self.get_parameter("robot").value

        descriptions_share_dir = get_package_share_directory("riptide_descriptions2")
        robot_config_subpath = os.path.join("config", self.robotName + ".yaml")
        self.configPath = os.path.join(descriptions_share_dir, robot_config_subpath)

        try:
            with open(self.configPath, "r") as config_file:
                config = yaml.safe_load(config_file)

                #set params from config
                self.declare_parameter("temperature_standard_dev", config["pressure_monitoring"]["temperature_standard_dev"])
                self.declare_parameter("pressure_standard_dev", config["pressure_monitoring"]["pressure_standard_dev"])
                self.declare_parameter("pressurization_tolerance", config["pressure_monitoring"]["pressurization_tolerance"])
                self.declare_parameter("hull_volume", config["hull_volume"])
                self.declare_parameter("leak_decay", config["pressure_monitoring"]["leak_decay"])
                self.declare_parameter("leak_init", config["pressure_monitoring"]["initial_leak"])

        except KeyError:

            self.get_logger().warn("Cannot find keys in config!")

            self.declare_parameter("temperature_standard_dev", 2)
            self.declare_parameter("pressure_standard_dev", .001)
            self.declare_parameter("pressurization_tolerance", .02)
            self.declare_parameter("hull_volume", .1)
            self.declare_parameter("leak_decay", 84000)
            self.declare_parameter("leak_init", .00001)


        except FileNotFoundError as e:

            self.get_logger().warn(f"Cannot open yaml {self.configPath}!")

            self.declare_parameter("temperature_standard_dev", 2)
            self.declare_parameter("pressure_standard_dev", .001)
            self.declare_parameter("pressurization_tolerance", .02)
            self.declare_parameter("hull_volume", .1)
            self.declare_parameter("leak_decay", 84000)
            self.declare_parameter("leak_init", .00001)


        self.temperature_standard_dev = self.get_parameter("temperature_standard_dev").value
        self.pressure_standard_dev = self.get_parameter("pressure_standard_dev").value
        self.pressurization_tolerance = self.get_parameter("pressurization_tolerance").value
        self.hull_volume = self.get_parameter("hull_volume").value
        self.leak_decay = self.get_parameter("leak_decay").value
        self.leak_init = self.get_parameter("leak_init").value

    def depressurize_callback(self, goal_handle):
        self.get_logger().info('Starting Depressurization')

        #clear the previous depressurization state if any
        self.depressurized_pvt = None
        self.depressurized_pvt_std = None
        self.depressurized_pvt_samples = None
        self.pvt_leak_rate_compensation = None

        #remove depressurization log
        self.remove_depressurization_log()

        
        #get the sampling time
        sample_time = goal_handle.request.sampling_time

        #set the node's sampling time
        self.sampling_time = sample_time

        #show sampling started on LEDs
        self.publish_led_states(LedStates.QUICK_ANALYZE.value)

        #add sample set to target
        self.collecting_sample_set = SampleSet(self.get_current_time_as_double())

        sample_start_time = self.get_current_time_as_double()

        #identify current pressure
        while(sample_start_time + sample_time > self.get_current_time_as_double()):
            sleep(.01)

        #decide if initial readings were okay
        initial_pressure = 0
        
        self.get_logger().info('Completed Initial Sampling')

        #check to make sure the sampler is actually getting filled
        if not self.collecting_sample_set.is_filled():
            self.get_logger().warn("Sample collcector is not filling! Please ensure sampling time is long enough and all publishers are working!")
            self.publish_led_states(LedStates.TEST_FAILED.value)
            self.turnoff_led_timer = self.create_timer(15, self.turn_off_leds, callback_group=self.general_callback_group)
            
            #quit here, something is not publishing
            self.get_logger().error(f"Cannot proceed with depressurization, someone doesn't want to talk. Pressure: {self.collecting_sample_set.get_average_pressure()}, Poac Temp Dev: {self.collecting_sample_set.get_ecage_temp_std_dev()}, CC Temp Dev: {self.collecting_sample_set.get_camera_cage_temp_std_dev()} ")
            goal_handle.abort()

            self.get_logger().info('Completed Initial Samplin2g')

            #show error on LEDs
            self.publish_led_states(LedStates.TEST_FAILED.value)
            self.turnoff_led_timer = self.create_timer(15, self.turn_off_leds, callback_group=self.general_callback_group)
        

            return Depressurize.Result()
            

        if(self.collecting_sample_set.get_pressure_std_dev() < self.pressure_standard_dev and self.collecting_sample_set.get_ecage_temp_std_dev() < self.temperature_standard_dev and self.collecting_sample_set.get_camera_cage_temp_std_dev() < self.temperature_standard_dev):
            initial_pressure = self.collecting_sample_set.get_average_pressure()

            #calculate and save the initial pvt
            self.initial_pvt = self.collecting_sample_set.get_pvt()

        else:
            self.get_logger().error(f"Cannot proceed with depressurization, the pressure or temperature is too unstable! Pressure Dev: {self.collecting_sample_set.get_pressure_std_dev()} Ecage Temp Dev: {self.collecting_sample_set.get_ecage_temp_std_dev()} Camera Cage Dev: {self.collecting_sample_set.get_camera_cage_temp_std_dev()}")
            goal_handle.abort()

            #show error on LEDs
            self.publish_led_states(LedStates.TEST_FAILED.value)
            self.turnoff_led_timer = self.create_timer(15, self.turn_off_leds, callback_group=self.general_callback_group)

            return Depressurize.Result()
        
        self.collecting_sample_set = None
        
        #determine the goal pressure
        target_pressure = initial_pressure - goal_handle.request.net_depressuization
        self.get_logger().info(f"Initial Hull Pressure: {initial_pressure} Target Hull Pressure: {target_pressure}")

        self.get_logger().info("Begin Lowering Pressure!")
        #wait while pressure is lowered
        sampled_final_pressure = initial_pressure
        sampled_final_pvt = 0 
        sampled_final_pvt_std = 0
        sampled_final_pvt_samples = 0
        while(sampled_final_pressure > target_pressure + self.pressurization_tolerance):

            #display leds to keep pumping
            self.publish_led_states(LedStates.KEEP_PUMPING.value)

            #wait untile pressure dips below target pressure
            last_pub_time = self.get_current_time_as_double()
            while(self.current_pressure > target_pressure):
                if(last_pub_time + PUBLISH_INTERVAL < self.get_current_time_as_double()):
                    #print state
                    self.get_logger().info(f"Continue Lowering Pressure. Current Hull Pressure {self.current_pressure}. Target Pressure: {target_pressure}.")

                    #publish feedback
                    feedback_msg = Depressurize.Feedback()
                    feedback_msg.current_pressure = self.current_pressure
                    goal_handle.publish_feedback(feedback_msg)

                    #update the publish time
                    last_pub_time = self.get_current_time_as_double()

            #get a clean pressure sample
            sampled_cleanly = False 

            while(not sampled_cleanly):

                #show LEDs in analyzing stte
                self.publish_led_states(LedStates.QUICK_ANALYZE.value)

                #run sampling to detect if pressurization is stable
                self.get_logger().info("Please wait! Collecting Pressure Sample Data!")

                sleep(30)

                self.get_logger().info("Collection beginning!")

                #add sample set to target
                self.collecting_sample_set = SampleSet(self.get_current_time_as_double())

                sample_start_time = self.get_current_time_as_double()

                #identify current pressure
                while(sample_start_time + sample_time > self.get_current_time_as_double()):
                    sleep(.01)
                    
                if not self.collecting_sample_set.is_filled():
                    self.get_logger().warn("Sample collcector is not filling! Please ensure sampling time is long enough and all publishers are working!")
                    self.publish_led_states(LedStates.TEST_FAILED.value)
                    self.turnoff_led_timer = self.create_timer(15, self.turn_off_leds, callback_group=self.general_callback_group)

                #check to see if samples are satisfactory
                if(self.collecting_sample_set.get_pressure_std_dev() < self.pressure_standard_dev and self.collecting_sample_set.get_ecage_temp_std_dev() < self.temperature_standard_dev and self.collecting_sample_set.get_camera_cage_temp_std_dev() < self.temperature_standard_dev):
                    sampled_final_pressure = self.collecting_sample_set.get_average_pressure()
                    sampled_final_pvt = self.collecting_sample_set.get_pvt()
                    sampled_final_pvt_std = self.collecting_sample_set.get_pvt_std()
                    sampled_final_pvt_samples = self.collecting_sample_set.get_num_samples()
                    sampled_cleanly = True

                    self.get_logger().info(f"Current Pressure is {sampled_final_pressure}. Samples were taken cleanly!")
                else:
                    self.get_logger().info("Samples were dirty. Retaking samples!")

                    #flash warning on LEDS
                    self.publish_led_states(LedStates.DIRTY_SAMPLES.value)
                    sleep(1)
                    
                self.collecting_sample_set = None

        if(sampled_final_pvt is None):
            self.get_logger().warn("Somehow samples went bye-bye. Quitting")
            goal_handle.abort()

            #show error on LEDs
            self.publish_led_states(LedStates.TEST_FAILED.value)
            self.turnoff_led_timer = self.create_timer(15, self.turn_off_leds, callback_group=self.general_callback_group)

            return Depressurize.Result()
                    
        #publish result
        result_msg = Depressurize.Result()
        result_msg.success = True
        result_msg.pvt = float(sampled_final_pvt)
        
        #record pvt characteristics
        self.depressurized_pvt = sampled_final_pvt
        self.depressurized_pvt_std = sampled_final_pvt_std
        self.depressurized_pvt_samples = sampled_final_pvt_samples
        self.pvt_leak_rate_compensation = 0

        #record the initial pressurization time
        self.initial_pressurization_time  = self.get_current_time_as_double()

        #write the depressurization log
        self.write_depressurization_log()

        #show success on leds
        self.publish_led_states(LedStates.PUMPING_COMPLETE.value)
        self.turnoff_led_timer = self.create_timer(15, self.turn_off_leds, callback_group=self.general_callback_group)

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

        if(self.collecting_sample_set.start_time + self.sampling_time > self.get_current_time_as_double()):
            #keep collecting and wait

            return
        
        if not self.collecting_sample_set.is_filled():
            self.get_logger().warn("Sample collector is not filling! Please ensure sampling time is long enough and all publishers are working!")
            self.do_not_getting_pressure()
            
            #reset and wait for samples
            self.collecting_sample_set = None
            return
        
        #get the stats on the current sample set
        current_pvt = self.collecting_sample_set.get_pvt()

        #publish the current pvt
        msg = Float32()
        msg.data = current_pvt

        #check to see if there is more air in the AUV than there should be
        maximum_pvt = self.check_pvt()

        if(current_pvt > maximum_pvt):
        #     #depressurization has been detected

            #probably make a bigger fuss than this
            self.get_logger().error(f"Depressurization Detected!!!!!!!!!  Max PVT: {maximum_pvt} Original PVT: {self.depressurized_pvt} New PVT: {current_pvt} Diff: {maximum_pvt - current_pvt}")

            #maybe hinge this on if the system is in the water?
            self.do_panic()

            #reset the system
            self.depressurized_pvt = None
            self.depressurized_pvt_std = None
            self.depressurized_pvt_samples = None
            self.pvt_leak_rate_compensation = None

            #remove depressurization log
            self.remove_depressurization_log()

            return
        else:
            self.get_logger().info(f"Depressurization Status: Max PVT: {maximum_pvt} Original PVT: {self.depressurized_pvt} New PVT: {current_pvt} Diff: {maximum_pvt - current_pvt}")
            self.current_pvt_w_state = current_pvt
        
            #adjust the rate compensator - only move down
            self.pvt_leak_rate_compensation += min(0, current_pvt + self.leak_init * (self.initial_pvt - self.depressurized_pvt) - maximum_pvt)

        #clear samples and start again
        self.collecting_sample_set = None
    
    def check_pvt(self):
        #check what the minimum should be at this points
        current_time = self.get_current_time_as_double()

        #calcuate the delta time
        delta_time = current_time - self.initial_pressurization_time 

        #calculate the acceptable amount of decay for this time periods
        return self.initial_pvt - (self.initial_pvt - self.depressurized_pvt) * exp(-delta_time / self.leak_decay) + self.leak_init * (self.initial_pvt - self.depressurized_pvt) + self.pvt_leak_rate_compensation


    def do_panic(self):
        #overwrite this with the panic behavoir
        self.get_logger().info("I am panic")
        
        self.publish_led_states(LedStates.TEST_FAILED.value)
        
        self.current_pvt_w_state = -1.0

    def do_not_getting_pressure(self):
        #by default, linking this to panic behavoir
        self.do_panic()
        
    def write_depressurization_log(self):
        #write the depressurization log

        data = dict()
        data["pvt"] = float(self.depressurized_pvt)
        data["pvt_std"] = float(self.depressurized_pvt_std)
        data["pvt_samples"] = float(self.depressurized_pvt_samples)
        data["sampling_time"] = self.sampling_time
        data["initial_pressurization_time"] = self.initial_pressurization_time
        data["initial_pvt"] = float(self.initial_pvt)
        data["leak_rate_compensation"] = self.pvt_leak_rate_compensation

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
            self.collecting_sample_set.add_ecage_sample(msg.data, self.get_current_time_as_double())

    def get_current_time_as_double(self):
        return self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1] * 1e-9
    
    def turn_off_leds(self):
        #callback for delayed LED turn off
        self.publish_led_states(0)
    
    def publish_led_states(self, state):
        msg = LedCommand()
        
        if not (self.turnoff_led_timer is None):
            self.turnoff_led_timer.cancel()
            self.turnoff_led_timer = None

        msg.target = LedCommand.TARGET_ALL

        #configure state
        if(state == 0):
                # off state
            msg.red = 0
            msg.blue = 0
            msg.green = 0

            msg.mode = LedCommand.MODE_SOLID

        elif(state == 1):
                # analyzing state
            msg.red = 255
            msg.blue = 191
            msg.green = 0

            msg.mode = LedCommand.MODE_FAST_FLASH

        elif(state == 2):
                # long term analyzing state
            msg.red = 255
            msg.blue = 191
            msg.green = 0

            msg.mode = LedCommand.MODE_BREATH

        elif(state == 3):
                # all good state
            msg.red = 0
            msg.blue = 0
            msg.green = 255

            msg.mode = LedCommand.MODE_BREATH

        elif(state == 4):
                #test failed state
            msg.red = 255
            msg.blue = 0
            msg.green = 0

            msg.mode = LedCommand.MODE_BREATH

        elif(state == 5):
                #keep pumping state
            msg.red = 0
            msg.blue = 0
            msg.green = 255

            msg.mode = LedCommand.MODE_SOLID

        elif(state == 6):
                #reduce pressure state
            msg.red = 255
            msg.blue = 0
            msg.green = 0

            msg.mode = LedCommand.MODE_SLOW_FLASH

        elif(state == 7):
                #pumping complete state
            msg.red = 0
            msg.blue = 0
            msg.green = 255

            msg.mode = LedCommand.MODE_FAST_FLASH

        elif(state == 8):
                #dirty sampling state
            msg.red = 255
            msg.blue = 0
            msg.green = 0

            msg.mode = LedCommand.MODE_FAST_FLASH

        else:
            self.get_logger().warn(f"Bad LED State Requested: {state}")
                    #bring everything to a known state
            self.depressurized_pvt = None
            self.depressurized_pvt_std = None
            self.depressurized_pvt_samples = None

            self.remove_depressurization_log()
        self.led_pub.publish(msg)
        
    def broadcast_pvt_state(self):
        #send the pressure state
        msg = Float32()
        
        if(self.current_pvt_w_state == -1.0):
            msg.data = -1.0
        elif not (self.current_pvt_w_state is None) and not (self.initial_pvt is None):
            msg.data = max(float((self.current_pvt_w_state - self.depressurized_pvt)/ (self.initial_pvt - self.depressurized_pvt)), 0.0)

        else:
            msg.data = 0.0
        
        self.pressure_pub.publish(msg)

    def reload_params(self):
        #reload ros2 parameters
        self.temperature_standard_dev = self.get_parameter("temperature_standard_dev").value
        self.pressure_standard_dev = self.get_parameter("pressure_standard_dev").value
        self.pressurization_tolerance = self.get_parameter("pressurization_tolerance").value
        self.hull_volume = self.get_parameter("hull_volume").value
        self.leak_decay = self.get_parameter("leak_decay").value
        self.leak_init = self.get_parameter("leak_init").value


def main(args=None):
    rclpy.init(args=args)

    pressure_monitor = PressureMonitor()

    executor = MultiThreadedExecutor()
    executor.add_node(pressure_monitor)

    try:
        pressure_monitor.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        pressure_monitor.get_logger().info('Keyboard interrupt, shutting down.\n')


if __name__ == '__main__':
    main()