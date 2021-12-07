#! /usr/bin/env python

import sys
import time
import rospy
import rosparam
from std_msgs.msg import String
from led_msgs.msg import led_blink, led_seq, led_solid, led_state


def talker():
	pub1 = rospy.Publisher("led_solid", led_solid, queue_size = 10)
	pub2 = rospy.Publisher("led_seq", led_seq, queue_size = 10)
	pub3 = rospy.Publisher("led_blink", led_blink, queue_size = 10)
	rospy.init_node('talker', anonymous=True)
	LED_solid_state = led_solid()
	LED_seq_state = led_seq()
	LED_blink_state = led_blink()

	rate = rospy.Rate(10) 

	while not rospy.is_shutdown():
		#solid state
		LED_solid_state.state = True
		LED_solid_state.led_count = 144
		LED_solid_state.r = 0
		LED_solid_state.g = 255
		LED_solid_state.b = 0
	
		#sequence
		LED_seq_state.state = True
		LED_seq_state.led_count = 144
		LED_seq_state.r = 255
		LED_seq_state.g = 0
		LED_seq_state.b = 0
		LED_seq_state.delay1 = 0.01
		LED_seq_state.delay2 = 0.5
		
		#blink
		LED_blink_state.state = True
		LED_blink_state.led_count = 144
		LED_blink_state.r = 0
		LED_blink_state.g = 255
		LED_blink_state.b = 0
		LED_blink_state.delay = 0.1
		

		
		
		pub1.publish(LED_solid_state)
		time.sleep(0.5)
		pub2.publish(LED_seq_state)
		#pub3.publish(LED_blink_state)
		rate.sleep()


if __name__ == '__main__':

	talker()




