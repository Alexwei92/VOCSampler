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
        self.sample = {
            'date': None,
            'time': None,
            'humidity': None,
            'temperature': None

        }

    def get_ros_parameter(self):
        port = rospy.get_param("~port", "/dev/ttyACM0")
        baud_rate = rospy.get_param("~baud_rate", 115200)
        self.serial = serial.Serial(port, baud_rate, timeout=1.0)

    def define_service(self):
        rospy.Service("command_sampler", CommandSampler, self.command_sampler)
        self.send_command_to_sampler = rospy.ServiceProxy("command_sampler", CommandSampler)

    def command_sampler(self, req):
        # s: Recording Start
        # r: Recording Stop
        if req.command == CommandSampler._request_class.COMMAND_MOTOR_START:
            if not self.is_active:
                self.serial.reset_input_buffer()
                self.serial.write(bytes(req.command, 'UTF-8'))
                self.is_active = True       
                rospy.loginfo("[voc_sampler]: Motor Started!")

        elif req.command == CommandSampler._request_class.COMMAND_MOTOR_STOP:
            if self.is_active:
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
                msg_list = msg.split(',')
                if len(msg_list) > 1:
                    self.sample['date'] = msg_list[0]
                    self.sample['time'] = msg_list[1]
                    self.sample['humidity'] = msg_list[2]
                    self.sample['temperature'] = msg_list[3]
                    for key in self.sample:
                        print(key + ": " + self.sample[key])
                
            self.rate.sleep()

    def clean(self):
        # stop sampler and serial when exit
        self.serial.write(
            bytes(CommandSampler._request_class.COMMAND_MOTOR_STOP, 'UTF-8')
        )
        self.serial.close()


if __name__ == "__main__":

    commander = SerialCommander()
    
    try:    
        commander.listen()
        rospy.spin()

    finally:
        commander.clean()