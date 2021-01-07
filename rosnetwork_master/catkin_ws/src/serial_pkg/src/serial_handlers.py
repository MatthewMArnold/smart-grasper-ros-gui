from grasper_msg.msg import *
from std_msgs.msg import Bool
import rospy
import struct

class MotorDebugInfoHandler:
    def __init__(self):
        self.pub = rospy.Publisher("serial/motorDebugFeedback", MotorDebugMessageFeedback, queue_size=1)

    def __call__(self, msg):
        try:
            data = struct.unpack("<iiqff", msg)
        except struct.error as e:
            rospy.logerr("invalid motor debug info message")
            return

        feedback = MotorDebugMessageFeedback()
        feedback.enc = data[0]
        feedback.strainGaugeADC = data[1]
        feedback.measuredStrainGaugeADC = data[2]
        feedback.jawPos = data[3]
        feedback.appliedForce = data[4]

        self.pub.publish(feedback)

class MotorInfoHandler:
    def __init__(self):
        self.pub = rospy.Publisher("serial/motorFeedback", MotorMessageFeedback, queue_size=1)

    def __call__(self, msg):
        try:
            data = struct.unpack("<ff", msg)
        except struct.error as e:
            rospy.logerr("invalid motor info message")
            return

        feedback = MotorMessageFeedback()
        feedback.jawPos = data[0]
        feedback.appliedForce = data[1]

        self.pub.publish(feedback)

class ThermistorMessageHandler:
    def __init__(self):
        self.pub = rospy.Publisher("serial/thermistorData", ThermistorMessage, queue_size=1)

    def __call__(self, msg):
        structuredMsg = ThermistorMessage()
        try:
            for i in len(50):
                structuredMsg.dataPoint.data[i] = struct.unpack("<f", msg[2 * i])
                structuredMsg.dataPoint.time[i] = struct.unpack("<L", msg[2 * (i + 1)])
        except struct.error as e:
            rospy.logerr("invalid thermistor data message")
            return

        self.pub.publish(structuredMsg)

class PulseOxMessageHandler:
    def __init__(self):
        self.pub = rospy.Publisher("serial/pulseOxData", PulseOxRxMessage, queue_size=1)

    def __call__(self, msg):
        structuredMsg = PulseOxRxMessage()
        try:
            for i in len(50):
                structuredMsg.dataPoint.data[i] = struct.unpack("<f", msg[2 * i])
                structuredMsg.dataPoint.time[i] = struct.unpack("<L", msg[2 * (i + 1)])
        except struct.error as e:
            rospy.logerr("invalid pulse ox data message")
            return

        self.pub.publish(structuredMsg)

class UltrasonicMessageHandler:
    def __init__(self):
        self.pub = rospy.Publisher("serial/ultrasonicData", UltrasonicDataMessage, queue_size=1)

    def __call__(self, msg):
        structuredMsg = UltrasonicDataMessage()
        try:
            for i in len(50):
                structuredMsg.dataPoint.data[i] = struct.unpack("<f", msg[2 * i])
                structuredMsg.dataPoint.time[i] = struct.unpack("<L", msg[2 * (i + 1)])
        except struct.error as e:
            rospy.logerr("invalid ultrasonic data message")
            return

        self.pub.publish(structuredMsg)

class ImpedanceMessageHandler:
    def __init__(self):
        self.pub = rospy.Publisher("serial/impedanceData", ImpedanceDataMessage, queue_size=1)

    def __call__(self, msg):
        structuredMsg = ImpedanceDataMessage()
        try:
            for i in len(50):
                structuredMsg.dataPoint.data[i] = struct.unpack("<f", msg[2 * i])
                structuredMsg.dataPoint.time[i] = struct.unpack("<L", msg[2 * (i + 1)])
        except struct.error as e:
            rospy.logerr("invalid impedance data message")
            return

        self.pub.publish(structuredMsg)

class MCUConnectedHandler
    def __init__(self):
        self.pub = rospy.Publisher("serial/mcuConnected", Bool, queue_size=1)
    
    def __call__(self, msg):
        structuredMsg = Bool()
        try:
            structuredMsg.data = struct.unpack("?", msg[0])
        except struct.error as e:
            rospy.logerr("invalid mcu connected message")
            return

        self.pub.publish(structuredMsg)

receiveHandlers = {
    0: MotorDebugInfoHandler(),
    1: MotorInfoHandler(),
    2: ThermistorMessageHandler(),
    3: PulseOxMessageHandler(),
    4: UltrasonicMessageHandler(),
    5: ImpedanceMessageHandler(),
    6: MCUConnectedHandler()
}

def handleRx(msgType, msg):
    if msgType not in receiveHandlers:
        rospy.logwarn_throttle(50, "Unhandled message type {0}".format(msgType))
        return
    receiveHandlers[msgType](msg)
