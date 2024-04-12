import rclpy
from rclpy.node import Node
from riptide_msgs2.msg import DshotCommand
from rclpy.qos import qos_profile_sensor_data

class ThrusterAlternater(Node):

    #wether the thrusters are currently spinning positive or not
    isPositive = True

    previousFrequency = 1.0

    def __init__(self):
        super().__init__("Thruster_Alternator")

        self.declare_parameter("Oscillation_Amplitude", 0)
        self.declare_parameter("Thrusters", [0])
        self.declare_parameter("Oscillation_Frequency", self.previousFrequency)

        #publisher
        self.rpmPub = self.create_publisher(DshotCommand, "/talos/command/thruster_rpm", qos_profile=qos_profile_sensor_data)

        #call back timer for the thruster alternation
        self.timer = self.create_timer(1/self.previousFrequency, self.alternate)

        #call timer to publish to thrusters
        self.create_timer(.02, self.publishToThrusters)

        self.rpmMsg = DshotCommand()


    def alternate(self):
        #get amplitude param
        amplitude = self.get_parameter("Oscillation_Amplitude")
        thursters = self.get_parameter("Thrusters")

        #rpm msg
        msg = DshotCommand()
        
        rpm = amplitude.value
        if(self.isPositive):
            rpm = -amplitude.value

        for thruster in thursters.value:
            msg.values[thruster] = rpm
        
        self.isPositive = not self.isPositive
        
        self.rpmMsg = msg

        #check if the frequency has changed
        frequency = self.get_parameter("Oscillation_Frequency").value
        if not (self.previousFrequency == frequency):
            self.previousFrequency = frequency

            #cancel the timer
            self.timer.cancel()

            #make a new timer at the frequency
            self.timer = self.create_timer(1 / frequency, self.alternate)

    def publishToThrusters(self):
        self.rpmPub.publish(self.rpmMsg)

def main(args=None):
    rclpy.init(args=args)
    node = ThrusterAlternater()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
