#!/usr/bin/env python
import rospy
import rosnode
from io import BytesIO
from time import sleep
from picamera import PiCamera


class PyCam:
    """Holds all parameters for one motor."""
    def __init__(self, channel, param_dict):
		self.camera = PiCamera()
		self.camera_publisher = rospy.Publisher()

	def pub(self):
		rate = rospy.Rate(50)
        while not rospy.is_shutdown():
			pass


if __name__ == '__main__':
    rospy.init_node('PyCam', anonymous=True)
    pycam = PyCam()
    roboteq.run()
