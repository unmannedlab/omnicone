#!/usr/bin/env python

import picamera

with picamera.PiCamera() as camera:
	camera.resolution = (640, 480)
	camera.start_recording('test_video_1.h264')
	camera.wait_recording(1)
	camera.stop_recording()
