from grasper_msg.msg import *
from std_msgs.msg import Bool
import rospy
import struct

class PeriodicInfoHandler:
    '''
    Handles a bunch of data that is sent over as a single package, repackaging
    the data and sending it off to respective publishers
    '''
    def __init__(self):
        self.thermistorPub = rospy.Publisher("serial/thermistorData", ThermistorMessage, queue_size=1)
        self.motorPub = rospy.Publisher("serial/motorFeedback", MotorMessageFeedback, queue_size=1)

    def __call__(self, msg):
        tempMsg = ThermistorMessage()
        motorMsg = MotorMessageFeedback()
        try:
            data = struct.unpack('<Lffi', msg)
        except struct.error as e:
            rospy.logerr("invalid thermistor data message")
            return

        tempMsg.dataPoint.data = data[1]
        tempMsg.dataPoint.time = data[0]

        motorMsg.time = data[0]
        motorMsg.appliedForce = data[2]
        motorMsg.jawPos = data[3]

        self.thermistorPub.publish(tempMsg)
        self.motorPub.publish(motorMsg)

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

class PulseOxMessageHandler:
    def __init__(self):
        self.pub = rospy.Publisher("serial/pulseOxData", PulseOxRxMessage, queue_size=1)

    def __call__(self, msg):
        structuredMsg = PulseOxRxMessage()
        try:
            for i in range(50):
                # insert janky parsing here :)
                tup = struct.unpack('<fL', msg[(8 * i) : (8 * i + 8)])
                structuredMsg.dataPoint[i].data = tup[0]
                structuredMsg.dataPoint[i].time = tup[1]
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
            for i in range(100):
                tup = struct.unpack('<fL', msg[(8 * i) : (8 * i + 8)])
                structuredMsg.dataPoint[i].data = tup[0]
                structuredMsg.dataPoint[i].time = tup[1]
            msgTail = struct.unpack('LL', msg[100 * 16:])
            structuredMsg.echoStart = msgTail[0]
            structuredMsg.echoEnd = msgTail[1]
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
            for i in range(50):
                tup = struct.unpack('<fL', msg[(8 * i) : (8 * i + 8)])
                structuredMsg.dataPoint[i].data = tup[0]
                structuredMsg.dataPoint[i].time = tup[1]
        except struct.error as e:
            rospy.logerr("invalid impedance data message")
            return

        self.pub.publish(structuredMsg)

class MCUConnectedHandler:
    def __init__(self):
        self.pub = rospy.Publisher('serial/mcuConnectedHandler', MCUConnectedMessage, queue_size=10)
    
    def __call__(self, msg):
        structuredMsg = MCUConnectedMessage()
        try:
            msgFormatted = struct.unpack("<?L", msg)
            structuredMsg.connected = msgFormatted[0]
            structuredMsg.time = msgFormatted[1]
        except struct.error as e:
            rospy.logerr("invalid mcu connected message")
            return

        self.pub.publish(structuredMsg)

receiveHandlers = {
    0: MotorDebugInfoHandler(),
    3: PulseOxMessageHandler(),
    4: UltrasonicMessageHandler(),
    5: ImpedanceMessageHandler(),
    6: MCUConnectedHandler(),
    7: PeriodicInfoHandler(),
}

def handleRx(msgType, msg):
    if msgType not in receiveHandlers:
        rospy.logwarn_throttle(50, "Unhandled message type {0}".format(msgType))
        return
    receiveHandlers[msgType](msg)
