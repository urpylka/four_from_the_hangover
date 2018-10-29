#!/usr/bin/env python

import rospy
from clever import srv

LED_COUNT = 6      # Number of LED pixels.
LED_PIN = 18      # GPIO pin connected to the pixels (18 uses PWM!).
#LED_PIN        = 10      # GPIO pin connected to the pixels (10 uses SPI /dev/spidev0.0).
LED_FREQ_HZ = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA = 10      # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 255     # Set to 0 for darkest and 255 for brightest
LED_INVERT = False   # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL = 0

import neopixel

rospy.init_node('Led')

neopixel.strip = neopixel.Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
neopixel.strip.begin()

def fun_led(req):
	if req.fun == "green":
		color = neopixel.Color(255,0,0)
		for i in range(0,LED_COUNT):
		        neopixel.strip.setPixelColor(i,color)
		        neopixel.strip.show()
	elif req.fun == "red":
                color = neopixel.Color(0,255,0)
                for i in range(0,LED_COUNT):
                        neopixel.strip.setPixelColor(i,color)
                        neopixel.strip.show()
	elif req.fun == "blue":
		color = neopixel.Color(0,0,255)
                for i in range(0,LED_COUNT):
                        neopixel.strip.setPixelColor(i,color)
                        neopixel.strip.show()
	elif req.fun == "off":
		color = neopixel.Color(0,0,0)
                for i in range(0,LED_COUNT):
                        neopixel.strip.setPixelColor(i,color)
                        neopixel.strip.show()

	return True

rospy.Service('led', srv.Lenta, fun_led)

def start_loop():
	r = rospy.Rate(10)
	while not rospy.is_shutdown():
		r.sleep()



start_loop()
