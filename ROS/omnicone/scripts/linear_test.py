#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D

rospy.init_node('pos_cmd', disable_signals=True)
pub = rospy.Publisher('robot_vel_goal', Pose2D, queue_size=10)

rpm = 50.0                                         # rev / minute
circumference = 3.14159 * 0.101                    # meters
linear_speed = rpm / 60 * circumference  # m/s



msg = Pose2D()

distance = 10 # meters

time = distance / linear_speed                         # seconds
rate = rospy.Rate(10)

for i in range(0,1):
    start_time = rospy.get_rostime()

    while (True):
        if (rospy.get_rostime() - start_time > rospy.Duration.from_sec(5.0) and
        rospy.get_rostime() - start_time < rospy.Duration.from_sec(5.0 + time)):
            msg.x = 0.0
            msg.y = 1.0 * linear_speed
            msg.theta = 0.0

        elif (rospy.get_rostime() - start_time > rospy.Duration.from_sec(10.0 + time) and
        rospy.get_rostime() - start_time < rospy.Duration.from_sec(10.0 + 2.0*time)):
            msg.x = 0.0
            msg.y = - 1.0 * linear_speed
            msg.theta = 0.0

        elif (rospy.get_rostime() - start_time > rospy.Duration.from_sec(10.0 + 2.0*time)):
            break

        else:
            msg.x = 0.0
            msg.y = 0.0
            msg.theta = 0.0

        pub.publish(msg)
        rate.sleep()
