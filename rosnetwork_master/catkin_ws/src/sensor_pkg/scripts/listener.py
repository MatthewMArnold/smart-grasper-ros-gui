#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
	
def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber('chatter', Float32, callback)
	
	#spin() keeps python from exiting until node is stopped
	rospy.spin()
	
if __name__ == '__main__':
	listener()
