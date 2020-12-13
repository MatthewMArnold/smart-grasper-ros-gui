#!/usr/bin/env python

from grasper_msg.msg import MotorRequestMessage
import rospy
import serial_handlers
import serial_parser
import time

MOTOR_REQUEST_TX_TYPE = 0

class SerialNode:
    def __init__(self):
        rospy.init_node("serial")
        self.socket = serial_parser.SerialSocket(serial_handlers.handleRx)
        self.motorListener = rospy.Subscriber("serial/motor", MotorRequestMessage, self.sendMotorRequest)

        rate = rospy.Rate(1000)
        c=0 #loopback test code

        while not rospy.is_shutdown():
            self.socket.listen()
            time.sleep(0.1)

    def sendMotorRequest(self, msg):
        data = struct.pack("<ff", msg.angle, msg.appliedForce)
        self.socket.sendMessage(data, MOTOR_REQUEST_TX_TYPE)

if __name__ == '__main__':
    SerialNode()
