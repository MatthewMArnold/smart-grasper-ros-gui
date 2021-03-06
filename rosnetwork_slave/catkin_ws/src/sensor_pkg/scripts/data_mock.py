#!/usr/bin/env python
import rospy
import random
from grasper_msg.msg import *
import math
from std_msgs.msg import Bool

def pulse_ox_talker():
    rospy.init_node('pulse_ox_talker')
    rate = rospy.Rate(5)
    pulseox_pub = rospy.Publisher('serial/pulseOxData', PulseOxRxMessage, queue_size=10)
    thermistor_pub = rospy.Publisher('serial/thermistorData', PulseOxRxMessage, queue_size=10)
    ultrasonic_pub = rospy.Publisher('serial/ultrasonicData', UltrasonicDataMessage, queue_size=10)
    impedance_pub = rospy.Publisher('serial/impedanceData', ImpedanceDataMessage, queue_size=10)
    mcu_connected_pub = rospy.Publisher('serial/mcuConnectedHandler', Bool, queue_size=10)
    serial_node_running_pub = rospy.Publisher('serial/serialNodeRunning', Bool, queue_size=1)
    force_pub = rospy.Publisher('serial/motorFeedback', MotorMessageFeedback, queue_size=10)
    j = 0
    while not rospy.is_shutdown():
        pulseox_num = 50 * math.sin(j * 1) + 50
        thermistor_num = 40.0 * float(random.randrange(0, 100)) / 100.0
        ultrasonic_num = float(random.randrange(0, 100)) / 100.0
        impedance_num = float(random.randrange(0, 100)) / 100.0

        pulseox_msg = PulseOxRxMessage()
        thermistor_msg = ThermistorMessage()
        ultrasonic_msg = UltrasonicDataMessage()
        impedance_msg = ImpedanceDataMessage()
        force_msg = MotorMessageFeedback()
        force_msg.appliedForce = float(random.randrange(0, 100)) / 10.0
        force_msg.jawPos = float(random.randrange(0, 100)) / 10.0
        force_msg.time = j
        mcu_msg = Bool()
        mcu_msg.data = True
        serial_running_msg = Bool()
        serial_running_msg.data = True

        for i in range(50):
            pulseox_msg.dataPoint[i].data = pulseox_num
            thermistor_msg.dataPoint[i].data = thermistor_num
            ultrasonic_msg.dataPoint[i].data = ultrasonic_num
            impedance_msg.dataPoint[i].data = impedance_num

            pulseox_msg.dataPoint[i].time = j
            thermistor_msg.dataPoint[i].time = j
            ultrasonic_msg.dataPoint[i].time = j
            impedance_msg.dataPoint[i].time = j

            j += 1

        pulseox_pub.publish(pulseox_msg)
        thermistor_pub.publish(thermistor_msg)
        ultrasonic_pub.publish(ultrasonic_msg)
        impedance_pub.publish(impedance_msg)
        mcu_connected_pub.publish(mcu_msg)
        serial_node_running_pub.publish(serial_running_msg)
        force_pub.publish(force_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        pulse_ox_talker()
    except rospy.ROSInterruptException:
        pass
