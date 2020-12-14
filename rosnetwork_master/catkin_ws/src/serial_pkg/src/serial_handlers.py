from grasper_msg.msg import *
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
        msg = ThermistorMessage()
        try:
            for i in len(50):
                msg.dataPoint.data[i] = struct.unpack("<f", msg[2 * i])
                msg.dataPoint.time[i] = struct.unpack("<L", msg[2 * (i + 1)])
        except struct.error as e:
            rospy.logerr("invalid thermistor data message")
            return

        self.pub.publish(msg)

class PulseOxMessageHandler:
    def __init__(self):
        self.pub = rospy.Publisher("serial/pulseOxData", PulseOxRxMessage, queue_size=1)

    def __call__(self, msg):
        msg = PulseOxRxMessage()
        try:
            for i in len(50):
                msg.dataPoint.data[i] = struct.unpack("<f", msg[2 * i])
                msg.dataPoint.time[i] = struct.unpack("<L", msg[2 * (i + 1)])
        except struct.error as e:
            rospy.logerr("invalid pulse ox data message")
            return

        self.pub.publish(msg)

class UltrasonicMessageHandler:
    def __init__(self):
        self.pub = rospy.Publisher("serial/ultrasonicData", UltrasonicDataMessage, queue_size=1)

    def __call__(self, msg):
        msg = UltrasonicDataMessage()
        try:
            for i in len(50):
                msg.dataPoint.data[i] = struct.unpack("<f", msg[2 * i])
                msg.dataPoint.time[i] = struct.unpack("<L", msg[2 * (i + 1)])
        except struct.error as e:
            rospy.logerr("invalid ultrasonic data message")
            return

        self.pub.publish(msg)

class ImpedanceMessageHandler:
    def __init__(self):
        self.pub = rospy.Publisher("serial/impedanceData", ImpedanceDataMessage, queue_size=1)

    def __call__(self, msg):
        msg = ImpedanceDataMessage()
        try:
            for i in len(50):
                msg.dataPoint.data[i] = struct.unpack("<f", msg[2 * i])
                msg.dataPoint.time[i] = struct.unpack("<L", msg[2 * (i + 1)])
        except struct.error as e:
            rospy.logerr("invalid impedance data message")
            return

        self.pub.publish(msg)

receiveHandlers = {
    0: MotorDebugInfoHandler(),
    1: MotorInfoHandler(),
    2: ThermistorMessageHandler(),
    3: PulseOxMessageHandler(),
    4: UltrasonicMessageHandler(),
    5: ImpedanceMessageHandler(),
}

def handleRx(msgType, msg):
    if msgType not in receiveHandlers:
        rospy.logwarn_throttle(50, "Unhandled message type {0}".format(msgType))
        return
    receiveHandlers[msgType](msg)
