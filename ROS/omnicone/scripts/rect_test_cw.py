#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D

rospy.init_node('pos_cmd', disable_signals=True)
pub = rospy.Publisher('robot_vel_goal', Pose2D, queue_size=10)

rpm = 50.0                                         # rev / minute
circumference = 3.14159 * 0.101                    # meters
linear_speed = rpm / 60 * circumference  # m/s



msg = Pose2D()

fwd_distance = 2 # meters
sde_distance = 1  # meters

fwd_time = fwd_distance / linear_speed  # seconds
sde_time = sde_distance / linear_speed  # seconds
rate = rospy.Rate(10)

for i in range(0,5):
    start_time = rospy.get_rostime()

    while (True):
        # Go Forward
        if (rospy.get_rostime() - start_time > rospy.Duration.from_sec(5.0) and
        rospy.get_rostime() - start_time < rospy.Duration.from_sec(5.0 + fwd_time)):
            msg.x = 0.0
            msg.y = 1.0 * linear_speed
            msg.theta = 0.0

        # Go Right
        elif (rospy.get_rostime() - start_time > rospy.Duration.from_sec(10.0 + fwd_time) and
        rospy.get_rostime() - start_time < rospy.Duration.from_sec(10.0 + fwd_time + sde_time)):
            msg.x = 1.0 * linear_speed
            msg.y = 0.0
            msg.theta = 0.0

        # Go Backward
        elif (rospy.get_rostime() - start_time > rospy.Duration.from_sec(15.0 + fwd_time + sde_time) and
        rospy.get_rostime() - start_time < rospy.Duration.from_sec(15.0 + 2.0*fwd_time + sde_time)):
            msg.x = 0.0
            msg.y = - 1.0 * linear_speed
            msg.theta = 0.0

        # Go Left
        elif (rospy.get_rostime() - start_time > rospy.Duration.from_sec(20.0 + 2.0*fwd_time + sde_time) and
        rospy.get_rostime() - start_time < rospy.Duration.from_sec(20.0 + 2.0*fwd_time + 2.0*sde_time)):
            msg.x = - 1.0 * linear_speed
            msg.y = 0.0
            msg.theta = 0.0

        elif (rospy.get_rostime() - start_time > rospy.Duration.from_sec(20.0 + 2.0*fwd_time + 2.0*sde_time)):
            break

        else:
            msg.x = 0.0
            msg.y = 0.0
            msg.theta = 0.0

        pub.publish(msg)
        rate.sleep()
