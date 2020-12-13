from grasper_msg.msg import MotorDebugMessageFeedback, MotorMessageFeedback
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

        # rospy.loginfo("motor debug message: enc={0}, straingauge={1}, measured load cell={2}, jawPos={3}, appiledForce={4}"
        #         .format(feedback.enc, feedback.strainGaugeADC, feedback.measuredStrainGaugeADC, feedback.jawPos, feedback.appliedForce))

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

        # rospy.loginfo("motor info message: jawPos={0}, appiledForce={1}".format(feedback.jawPos, feedback.appliedForce))

        self.pub.publish(feedback)

receiveHandlers = {
    0: MotorDebugInfoHandler(),
    1: MotorInfoHandler()
}

def handleRx(msgType, msg):
    if msgType not in receiveHandlers:
        rospy.logwarn_throttle(50, "Unhandled message type {0}".format(msgType))
        return
    receiveHandlers[msgType](msg)
