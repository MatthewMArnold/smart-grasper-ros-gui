#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class CameraNode:
    def __init__(self):
        rospy.init_node('camera')
        self.cameraPublisher = rospy.Publisher('camera/cameraFrames', Image, queue_size=10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if not self.update():
                return

    def update(self):
        ret, frame = self.cap.read()
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return False

        try:
            self.cameraPublisher.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        except CvBridgeError as e:
            print(e)

        return True

def main():
    node = CameraNode()

if __name__ == "__main__":
    main()
