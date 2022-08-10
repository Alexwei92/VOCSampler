#!/usr/bin/env python
from tkinter import Variable
import serial
import rospy

from voc_sampler.srv import CommandSampler


class SerialCommander(object):
    def __init__(self):
        rospy.init_node("voc_sampler")
        self.rate = rospy.Rate(30)
        rospy.loginfo("[voc_sampler]: Node Started!")

        self.init_variable()
        self.get_ros_parameter()
        self.define_service()

    def init_variable(self):
        self.is_active = False

    def get_ros_parameter(self):
        port = rospy.get_param("~port", "/dev/ttyACM0")
        baud_rate = rospy.get_param("~baud_rate", 115200)
        self.serial = serial.Serial(port, baud_rate)

    def define_service(self):
        rospy.Service("command_sampler", CommandSampler, self.command_sampler)
        self.send_command_to_sampler = rospy.ServiceProxy("command_sampler", CommandSampler)

    def command_sampler(self, req):
        # s: Recording Start
        # r: Recording Stop
       
        if req.command == CommandSampler._request_class.COMMAND_MOTOR_START:
            self.serial.write(bytes(req.command, 'UTF-8'))
            self.is_active = True       
            rospy.loginfo("[voc_sampler]: Motor Started!")

        elif req.command == CommandSampler._request_class.COMMAND_MOTOR_STOP:
            self.serial.write(bytes(req.command, 'UTF-8'))
            self.is_active = False
            rospy.loginfo("[voc_sampler]: Motor Stoped!")
        
        else:
            rospy.logwarn("[voc_sampler]: Unrecognized command to sampler!")
        
        return True

    def listen(self):
        while not rospy.is_shutdown():
            if self.is_active:
                msg = self.serial.readline().decode("utf-8")
                print(msg)
            
            self.rate.sleep()


if __name__ == "__main__":

    commander = SerialCommander()
    commander.listen()
    rospy.spin()