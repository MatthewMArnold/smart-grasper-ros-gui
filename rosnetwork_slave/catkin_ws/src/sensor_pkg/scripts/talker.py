#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from beginner_tutorials.msg import Sensor
def talker():
	pub = rospy.Publisher('chatter', Sensor, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10) #10Hz
	while not rospy.is_shutdown():
		msg = Sensor()
		msg.ir = 3.0
		msg.photo = 4.0
		msg.therm = 5.0
		rospy.loginfo(msg)
		pub.publish(msg)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
	
