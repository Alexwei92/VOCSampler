#!/usr/bin/env python
import serial
import rospy

from voc_sampler.srv import CommandSampler


class SerialCommander(object):
    def __init__(self):
        rospy.init_node("voc_sampler")
        rospy.loginfo("[voc_sampler]: Node Started!")

        # Get ROS parameters
        port = rospy.get_param("~port", "/dev/ttyACM0")
        baud_rate = rospy.get_param("~baud_rate", 115200)
        self.serial = serial.Serial(port, baud_rate)    

        # ROS Service
        rospy.Service("command_sampler", CommandSampler, self.command_sampler)
        
        # ROS Service Proxy
        self.send_command_to_sampler = rospy.ServiceProxy("command_sampler", CommandSampler)

    def command_sampler(self, req):
        # s: Recording Start
        # r: Recording Stop
        self.serial.write(bytes(req.command, 'UTF-8'))
       
        if req.command == CommandSampler._request_class.COMMAND_MOTOR_START:
            rospy.loginfo("[voc_sampler]: Motor Started!")

        elif req.command == CommandSampler._request_class.COMMAND_MOTOR_STOP:
            rospy.loginfo("[voc_sampler]: Motor Stoped!")
        
        else:
            rospy.logwarn("[voc_sampler]: Unrecognized command to sampler!")
        
        return True


if __name__ == "__main__":

    commander = SerialCommander()
    rospy.spin()