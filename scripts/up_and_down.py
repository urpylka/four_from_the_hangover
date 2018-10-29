#!/usr/bin/env python

import rospy
from clever import srv

rospy.init_node('up_&_down_node')

def target(req):
	if req == "red":
		color = neopixel.Color(255,0,0)
		for i in range(0,LED_COUNT):
		        neopixel.strip.setPixelColor(i,color)
		        neopixel.strip.show()
	elif req == "blue":
                color = neopixel.Color(0,255,0)
                for i in range(0,LED_COUNT):
                        neopixel.strip.setPixelColor(i,color)
                        neopixel.strip.show()
	print req.fun
	return True

rospy.Service('up_&_down_srv', srv.UpDown, target)

def start_loop():
	r = rospy.Rate(10)
	while not rospy.is_shutdown():
		r.sleep()



