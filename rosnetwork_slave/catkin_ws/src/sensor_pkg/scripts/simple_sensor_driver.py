#!/usr/bin/env python
#@Author: premwong@uw.edu
#driver for grasper sensor board. Publishes to ros topic

import rospy
from sensor_pkg.msg import Sensor
from std_msgs.msg import Float32
import piplates.DAQCplate as DAQC
import time 
from enum import Enum
ADDRESS = 0

class Led(Enum):
	RED = 0
	GREEN = 1
	BLUE = 2
	
class Channel(Enum):
	IR = 0
	THERM = 1
	PHOTO = 2
	


def set_led(led_color, on_off=1):
	if led_color == 3:
		DAQC.clrDOUTbit(ADDRESS, 2)
	elif on_off:
		DAQC.setDOUTbit(ADDRESS, led_color)
	else:
		DAQC.clrDOUTbit(ADDRESS, led_color)
		
def read_adc(channel):
	return DAQC.getADC(ADDRESS, channel)

def talker():
	set_led(0, 1)
	set_led(1, 0)
	set_led(2, 0)
	led_cycle = 0
	cycle_toggle = 1
	CYCLE_WAIT_TIME = 1
	current_time = 0
	current_time = time.time()
	pub = rospy.Publisher('sensor_chatter', Sensor, queue_size=10)
	rospy.init_node('sensor_talker', anonymous=True)
	rate = rospy.Rate(40) #10Hz
	while not rospy.is_shutdown():
		msg = Sensor()
		msg.ir = read_adc(0)
		msg.therm = read_adc(1)
		msg.photo = read_adc(2)
		rospy.loginfo(msg)
		pub.publish(msg)
		if (cycle_toggle):
			if (time.time() - current_time > CYCLE_WAIT_TIME):
				set_led(led_cycle % 4, 0)
				led_cycle += 1
				set_led(led_cycle % 4)
				current_time = time.time()
				
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
	
