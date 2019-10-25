#!/usr/bin/env python

import rospy
import open_base
import math
from std_msgs.msg import Float32, Float64, Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, PoseWithCovariance, TwistWithCovariance

from math import pi

class EncoderOdom:

    def __init__(self):
        # Velocity Commander Initialization
        # Description:
        #     Function initializes the velocity commander by creating the node,
        #     subscribers, and publishers. Additionally, the parameters and
        #     persistent variables are set/initialized to zero.

        rospy.init_node('EncoderOdom')

        # Paramters
        self.circumference = pi * 0.101                     # meters
        self.linear_to_rot = 2 * pi / self.circumference    # rad / m
        self.command_timeout = 1.0                          # sec

        # create the subscriber for the encoder_pulses_per_revolution
        self.left_enc_sub  = rospy.Subscriber( '/left_joint_velocity_controller/absolute_encoder_count', Int32, self.updateEnc_left)
        self.back_enc_sub  = rospy.Subscriber( '/back_joint_velocity_controller/absolute_encoder_count', Int32, self.updateEnc_back)
        self.right_enc_sub = rospy.Subscriber('/right_joint_velocity_controller/absolute_encoder_count', Int32, self.updatePos)

        # create publisher for position estimate
        self.odom_pub = rospy.Publisher('EncoderOdom/Odometry', Odometry, queue_size=10)
        self.pose_pub = rospy.Publisher('EncoderOdom/Pose', PoseWithCovariance, queue_size=10)
        self.twst_pub = rospy.Publisher('EncoderOdom/Twist', TwistWithCovariance, queue_size=10)

        # set/initialize persistent variables
        self.enc_init = [False, False, False]
        self.enc_prev = [0.0, 0.0, 0.0]
        self.enc_curr = [0.0, 0.0, 0.0]
        self.d_pos_est = Pose2D(0,0,0)

        self.odom = Odometry()

        self.odom.pose.covariance = [   0.01, 0, 0, 0, 0, 0, \
                                        0, 0.01, 0, 0, 0, 0, \
                                        0, 0, 0.01, 0, 0, 0, \
                                        0, 0, 0, 0.01, 0, 0, \
                                        0, 0, 0, 0, 0.01, 0, \
                                        0, 0, 0, 0, 0, 0.01  ]

        self.odom.twist.covariance = [  0.01, 0, 0, 0, 0, 0, \
                                        0, 0.01, 0, 0, 0, 0, \
                                        0, 0, 0.01, 0, 0, 0, \
                                        0, 0, 0, 0.01, 0, 0, \
                                        0, 0, 0, 0, 0.01, 0, \
                                        0, 0, 0, 0, 0, 0.01  ]

    def updateEnc_left(self, msg):
        # Left Motor Encoder Callback
        # Description
        #     Function updates the internal memory of the left encoder counter.
        #     If the variable has not been initialized, the first value is assigned
        #     to both the previous and current encoder value.
        self.enc_curr[0] = msg.data
        if not self.enc_init[0]:
            self.enc_prev[0] = self.enc_curr[0]
            self.enc_init[0] = True

    def updateEnc_back(self, msg):
        # Back Motor Encoder Callback
        # Description
        #     Function updates the internal memory of the back encoder counter.
        #     If the variable has not been initialized, the first value is assigned
        #     to both the previous and current encoder value.
        self.enc_curr[1] = msg.data
        if not self.enc_init[1]:
            self.enc_prev[1] = self.enc_curr[1]
            self.enc_init[1] = True


    def updatePos(self, msg):
        # Right Motor Encoder Callback
        # Description
        #     Function updates the internal memory of the right encoder counter.
        #     If the variable has not been initialized, the first value is assigned
        #     to both the previous and current encoder value. Additionally,
        #     function integrates the wheel rotations and uses the omni FK to
        #     estimate a change in position since the start of the program.
        self.enc_curr[2] = msg.data
        if not self.enc_init[2]:
            self.enc_prev[2] = self.enc_curr[2]
            self.enc_init[2] = True
            self.time_prev = rospy.get_time()

        self.time_curr = rospy.get_time()

        # num encoder counts / (4 counts / cycle ) / ( 7 * 27 cycles / rev) (2pi rad / rev)
        theta_l = -float(self.enc_curr[0] - self.enc_prev[0]) / 4 / (7 * 27) * (2 * pi)
        theta_b = -float(self.enc_curr[1] - self.enc_prev[1]) / 4 / (7 * 27) * (2 * pi)
        theta_r = -float(self.enc_curr[2] - self.enc_prev[2]) / 4 / (7 * 27) * (2 * pi)


        # v_linear = omega * d / 2
        D1 = theta_l * 0.101 / 2
        D2 = theta_b * 0.101 / 2
        D3 = theta_r * 0.101 / 2

        # Standard transformation from
        delta_x = (2*D2 - D1 - D3) / 3
        delta_y = math.sqrt(3) * (D3 - D1) / 3
        delta_w = (D1 + D2 + D3)/(3 * 0.1905)

        # Rotation to World Frame
        delta_x_w = delta_x*math.cos(delta_w/2 + self.d_pos_est.theta) - delta_y*math.sin(delta_w/2 + self.d_pos_est.theta)
        delta_y_w = delta_x*math.sin(delta_w/2 + self.d_pos_est.theta) + delta_y*math.cos(delta_w/2 + self.d_pos_est.theta)

        # Element-wise assign or else assigned by reference (very bad)
        self.enc_prev[0] = self.enc_curr[0]
        self.enc_prev[1] = self.enc_curr[1]
        self.enc_prev[2] = self.enc_curr[2]

        self.d_pos_est.theta = delta_w + self.d_pos_est.theta

        self.odom.pose.pose.position.x = delta_x_w + self.odom.pose.pose.position.x
        self.odom.pose.pose.position.y = delta_y_w + self.odom.pose.pose.position.y
        self.odom.pose.pose.position.z = 0
        self.odom.pose.pose.orientation.x = 0
        self.odom.pose.pose.orientation.y = 0
        self.odom.pose.pose.orientation.z = math.sin(self.d_pos_est.theta/2)
        self.odom.pose.pose.orientation.w = math.cos(self.d_pos_est.theta/2)

        self.odom.twist.twist.linear.x = delta_x_w / (self.time_curr - self.time_prev)
        self.odom.twist.twist.linear.y = delta_y_w / (self.time_curr - self.time_prev)
        self.odom.twist.twist.linear.z = 0
        self.odom.twist.twist.angular.x = 0
        self.odom.twist.twist.angular.y = 0
        self.odom.twist.twist.angular.z = delta_w / (self.time_curr - self.time_prev)

        self.time_curr = self.time_prev

    def run(self):
        rate = rospy.Rate(10) # 10 Hz

        while(not rospy.is_shutdown()):

            self.odom_pub.publish( self.odom )
            self.pose_pub.publish( self.odom.pose )
            self.twst_pub.publish( self.odom.twist )

            rate.sleep()


if __name__ == '__main__':
    EncoderOdom = EncoderOdom()
    EncoderOdom.run()
