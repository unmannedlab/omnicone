#!/usr/bin/env python

import rospy
from io import BytesIO
from time import sleep
from picamera import PiCamera

if __name__ == "__main__":
	rospy.init_node('camera_preview')
	camera = PiCamera()
	camera.start_preview(fullscreen=False, window=(0, 250, 250, 250))
	
	rospy.spin()	

	camera.stop_preview()
	camera.close()
