#!/usr/bin/env python

import os, sys, rospkg

import rospy
from geometry_msgs.msg import Pose2D


rospy.init_node('gui_cmd', disable_signals=True)
pub = rospy.Publisher('UBX/gui_cmd', Pose2D, queue_size=10)


msg = Pose2D()

rpm = 50.0                                         # rev / minute
circumference = 3.14159 * 0.101                    # meters
linear_speed = rpm / 60 * circumference  # m/s

distance = 10 # meters

time = distance / linear_speed                         # seconds

rate = rospy.Rate(10)

for i in range(1):
    start_time = rospy.get_rostime()
    run_switch = True

    while (run_switch):
        if   (rospy.get_rostime() - start_time > rospy.Duration.from_sec(5.0) and
              rospy.get_rostime() - start_time < rospy.Duration.from_sec(10.0 + 1*time)):
            msg.x = 10.0
            msg.y = 0.0
            msg.theta = 0.0

        elif (rospy.get_rostime() - start_time > rospy.Duration.from_sec(10.0 + 1*time) and
              rospy.get_rostime() - start_time < rospy.Duration.from_sec(15.0 + 2*time)):
            msg.x = 10.0
            msg.y = 10.0
            msg.theta = 0.0

        elif (rospy.get_rostime() - start_time > rospy.Duration.from_sec(15.0 + 2*time) and
              rospy.get_rostime() - start_time < rospy.Duration.from_sec(20.0 + 3*time)):
            msg.x = 00.0
            msg.y = 10.0
            msg.theta = 0.0

        elif (rospy.get_rostime() - start_time > rospy.Duration.from_sec(25.0 + 3*time) and
              rospy.get_rostime() - start_time < rospy.Duration.from_sec(30.0 + 4*time)):
            msg.x = 0.0
            msg.y = 0.0
            msg.theta = 0.0
            run_switch = False

        else:
            msg.x = 0.0
            msg.y = 0.0
            msg.theta = 0.0


        pub.publish(msg)
        rate.sleep()
