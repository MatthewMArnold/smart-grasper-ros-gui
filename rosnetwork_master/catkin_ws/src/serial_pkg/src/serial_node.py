#!/usr/bin/env python

from grasper_msg.msg import MotorRequestMessage, SensorRequestMessage
import rospy
import serial_handlers
import serial_parser
import time

MOTOR_REQUEST_TX_TYPE = 0
SENSOR_ENABLE_TX_TYPE = 1

class SerialNode:
    def __init__(self):
        rospy.init_node("serial")
        self.socket = serial_parser.SerialSocket(serial_handlers.handleRx)
        self.motorListener = rospy.Subscriber("serial/motor", MotorRequestMessage, self.sendMotorRequest)
        self.sensorEnableListener = rospy.Subscriber("serial/sensorEnable", SensorRequestMessage)

        rate = rospy.Rate(1000)

        while not rospy.is_shutdown():
            self.socket.listen()
            time.sleep(0.1)

    def sendMotorRequest(self, msg):    
        data = struct.pack("<f??", msg.angle, msg.enableMotorController, msg.measureForce)
        self.socket.sendMessage(data, MOTOR_REQUEST_TX_TYPE)

    def sendSensorEnableRequest(self, msg):
        data = struct.pack("????", msg.enablePulseOx, msg.enableTemperature, msg.enableVelocityOfSound, msg.enableImpedance)
        self.socket.sendMessage(data, )

if __name__ == '__main__':
    SerialNode()
