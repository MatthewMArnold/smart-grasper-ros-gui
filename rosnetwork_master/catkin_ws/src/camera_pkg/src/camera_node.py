#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from grasper_msg.msg import CameraRequestMessage
from threading import Lock

class CameraNode:
    def __init__(self):
        rospy.init_node('camera')
        self.cameraPublisher = rospy.Publisher('camera/cameraFrames', Image, queue_size=10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        self.requestSubscriber = rospy.Subscriber(
            'camera/requestCameraFrames',
            CameraRequestMessage,
            self.signalMessageCapture)
        self.messagesToHandle = []
        self.cameraRequestLock = Lock()

        rate = rospy.Rate(1000)

        while not rospy.is_shutdown():
            if not self.update():
                return
            rate.sleep()

    def signalMessageCapture(self, msg):
        self.cameraRequestLock.acquire()
        self.messagesToHandle.append({
            'cameraFrames': msg.cameraFramesToCapture,
            'rate': msg.cameraFrameCaptureRate
        })
        self.cameraRequestLock.release()
        
    def update(self):
        if len(self.messagesToHandle) == 0:
            return True

        # Check if message needs updating
        self.cameraRequestLock.acquire()
        # Grab first message, slice off first message from list
        msg = self.messagesToHandle[0]
        self.messagesToHandle = self.messagesToHandle[1:]
        self.cameraRequestLock.release()

        rate = rospy.Rate(msg['rate'])

        for i in range(msg['cameraFrames']):
            if not self.processSingleFrame():
                return False
            rate.sleep()

        return True

    def processSingleFrame(self):
        ret, frame = self.cap.read()
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.logerr('cv2.waitKey failed')
            return False

        try:
            self.cameraPublisher.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)
            return False

        return True

def main():
    node = CameraNode()

if __name__ == "__main__":
    main()
