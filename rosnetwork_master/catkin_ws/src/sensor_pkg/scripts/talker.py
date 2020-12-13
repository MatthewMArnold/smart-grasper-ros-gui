#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

def talker():
	pub = rospy.Publisher('sensor_chatter', Float32, queue_size=10)
	rospy.init_node('talker')
	rate = rospy.Rate(10) #10Hz
	while not rospy.is_shutdown():
		hello_str = 5.0
		rospy.loginfo(hello_str)
		pub.publish(hello_str)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
	
