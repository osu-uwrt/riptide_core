import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from riptide_msgs2.msg import DshotRPMFeedback

#the purpose of this node is to concatinate all thruster data into a single message that includes the latest rpm from each thruster

#TODO figure out real topic names

class Thruster_RPM_Echo(Node):

    #the rpms - most currrent updates
    thruster_rpms = [0,0,0,0,0,0,0,0]

    #if the thruster has yet to have been logged
    thruster_tracked = [False,False,False,False,False,False,False,False]

    #mask of valid thrusters - same scheme as firmware
    valid_mask = 0

    def __init__(self):
        #init super node
        super().__init__("Thruster_RPM_Echo")

        #sub to rpm stream from firmware
        self.sub = self.create_subscription(DshotRPMFeedback, "state/thrusters/rpm", callback=self.cb, qos_profile=qos_profile_sensor_data)

        #pub to new topic
        self.pub = self.create_publisher(DshotRPMFeedback, "state/thrusters/rpm_complete", qos_profile=qos_profile_sensor_data)

    def cb(self, msg: DshotRPMFeedback):
        #the call back from when a new msg is published to the topic

        #pick apart valid thruster rpm
        if(self.valid_mask < 255):
            #if thrusters have yet to have been tracked

            #go through each bit
            n = 7
            while(n >= 0):

                #check to see if the msg has data on thruster n
                if(msg.rpm_valid_mask >= 2**n):

                    #check to see if the thruster has been logged yet
                    if(self.thruster_tracked[n] == False):

                        self.thruster_tracked[n] = True
                        self.valid_mask += 2**n

                    self.thruster_rpms[n] = msg.rpm[n]

                    msg.rpm_valid_mask -= 2**n

                n -= 1

        else:

            #go through each bit
            n = 7
            while(n >= 0):
                #stop checking mask

                #check to see if the msg has data on thruster n
                if(msg.rpm_valid_mask >= 2**n):
                    self.thruster_rpms[n] = msg.rpm[n]

                    msg.rpm_valid_mask -= 2**n

                n -= 1

        #the message to publish back 
        msgOut = DshotRPMFeedback()
        msgOut.rpm_valid_mask = self.valid_mask
        msgOut.rpm = [int(self.thruster_rpms[0]),
                      int(self.thruster_rpms[1]),
                      int(self.thruster_rpms[2]),
                      int(self.thruster_rpms[3]),
                      int(self.thruster_rpms[4]),
                      int(self.thruster_rpms[5]),
                      int(self.thruster_rpms[6]),
                      int(self.thruster_rpms[7]),]

        self.pub.publish(msgOut)





def main(args=None):
    rclpy.init(args=args)
    node = Thruster_RPM_Echo()
    rclpy.spin(node)


if __name__ == "__main__":
    main()