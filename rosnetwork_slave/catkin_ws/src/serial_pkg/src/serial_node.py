#!/usr/bin/env python

from grasper_msg.msg import MotorRequestMessage, SensorRequestMessage
from std_msgs.msg import Bool
import rospy
import serial_handlers
import serial_parser
import time
import struct

MOTOR_REQUEST_TX_TYPE = 0
SENSOR_ENABLE_TX_TYPE = 1

class SerialNode:
    def __init__(self):
        rospy.init_node("serial")
        self.socket = serial_parser.SerialSocket(serial_handlers.handleRx)
        self.motorListener = rospy.Subscriber("serial/motor", MotorRequestMessage, self.sendMotorRequest)
        self.sensorEnableListener = rospy.Subscriber("serial/sensorEnable", SensorRequestMessage)
        self.serialEnabledPublisher = rospy.Publisher("serial/serialNodeRunning", Bool, queue_size=1)

        rate = rospy.Rate(1000)

        counter = 0

        while not rospy.is_shutdown():
            counter += 1
            if counter % 100 == 1:
                msg = True
                self.serialEnabledPublisher.publish(msg)
            self.socket.listen()
            rate.sleep()

    def sendMotorRequest(self, msg):
        data = struct.pack("<f??", msg.appliedForce, msg.enableMotorController, msg.measureForce)
        self.socket.sendMessage(data, MOTOR_REQUEST_TX_TYPE)

    def sendSensorEnableRequest(self, msg):
        data = struct.pack(
            "????cccc",
            msg.enablePulseOx,
            msg.enableTemperature,
            msg.enableVelocityOfSound,
            msg.enableImpedance,
            msg.pulseoxIndex,
            msg.temperatureIndex,
            msg.velocityOfSoundIndex,
            msg.impedanceIndex)

        self.socket.sendMessage(data, SENSOR_ENABLE_TX_TYPE)

if __name__ == '__main__':
    SerialNode()
