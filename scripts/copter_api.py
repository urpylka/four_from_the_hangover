#!/usr/bin/env python

import rospy
from clever import srv
import time
from math import fabs,sqrt
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseArray

rospy.init_node('hack')
homeX = 4.5
homeY = 4.5

flyVosem = False
detected = False

led = rospy.ServiceProxy('/led', srv.Lenta)
navigate = rospy.ServiceProxy('/navigate', srv.Navigate)
get_telemetry = rospy.ServiceProxy('/get_telemetry',srv.GetTelemetry)
set_position = rospy.ServiceProxy('/set_position', srv.SetPosition)

def get_distance(x1, y1, z1, x2, y2, z2):
    return sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)

def hundle_detected(req):
	global set_position, detected
	if detected:
		set_position(x = req.poses.position.x, y = req.poses.position.y, z = 1.5, frame_id='aruco_map')

rospy.Subscriber('/detected_object', PoseArray, hundle_detected)

#Vzlet
def takeOff(req):	
	global navigate, led,time, get_telemetry, homeX, homeY	, startVosem, time
	led('green')	
	navigate(x=0.0,y=0.0, z=1, speed=0.5, yaw=1.5, frame_id='fcu_horiz', update_frame=False, auto_arm=True)
	time.sleep(5)
	navigate(x=homeX, y=homeY, z=1, yaw=1.5, speed=0.5, frame_id='aruco_map', update_frame=True, auto_arm=False)
	time.sleep(5)
	startVosem()
	return True

def stopVosem():
	global flyVosem
	flyVosem = False


def navigateWithWait(x,y,z,frame_id, yaw = 0, acc = 0.2, speed=0.5, auto_arm = False):
	global get_telemetry, navigate,get_distance
	telem = get_telemetry()
	start_dist = get_distance(telem.x, telem.y, telem.z, x, y, z)
	navigate(x=x,y=y,z=z,frame_id=frame_id,speed=speed,yaw=yaw, auto_arm = auto_arm)
	while True:
		telem = get_telemetry()
		if start_dist - get_distance(telem.x, telem.y, telem.z, x, y, z) < acc:
                        print "Yes"
			break
	return True		



def startVosem():
	global flyVosem, navigateWithWait, get_telemtry, homeX, homeY, start, time, detected
	detected = False
	navigateWithWait(x=homeX, y = homeY, z=1.5, yaw=1.5, frame_id='aruco_map')
	flyVosem = True
	start = time.time()


def detected(req):
	global led, navigate, stopVosem
	stopVosem()
	detected = True
	led('blue')
	return True

def home(req):
	global navigateWithWait, Trigger, led, rospy, homeX, homeY, stopVosem
	stopVosem()
	led('red')
	navigateWithWait(x=homeX, y = homeY, z = 0.4, yaw=1.5, frame_id='aruco_map')
	time.sleep(5)
	lands = rospy.ServiceProxy('land', Trigger)
	lands()
	led('off')
	return True

rospy.Service('take_off', srv.TakeOff, takeOff)
rospy.Service('home', srv.Home, home)
rospy.Service('detected', srv.Detected, detected)




def loop():
	global rospy
	while not rospy.is_shutdown():
		if(flyVosem):
			t = (time.time()-start)/10
			x = homeX + cos(t)*6
			y = homeY + sin(2*t)*3
			set_position(x = x, y = y, z = 1.5,yaw=1.5,frame_id='aruco_map')
		rospy.spin()

print "go"
loop()
