#!/usr/bin/env python

import rospy
import open_base
import math
import numpy as np
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist, Pose2D

from math import pi
class EKF_omnicone:


    def __init__(self):

        rospy.init_node('EKF')

        # Paramters
        self.circumference = pi * 0.101                     # meters
        self.linear_to_rot = 2 * pi / self.circumference    # rad / m
        self.command_timeout = 1.0                          # sec


        # create the subscriber for the encoder_pulses_per_revolution
        self.left_enc_sub  = rospy.Subscriber( '/left_joint_velocity_controller/absolute_encoder_count', Int32, self.updateEnc_left)
        self.back_enc_sub  = rospy.Subscriber( '/back_joint_velocity_controller/absolute_encoder_count', Int32, self.updateEnc_back)
        self.right_enc_sub = rospy.Subscriber('/right_joint_velocity_controller/absolute_encoder_count', Int32, self.predict)


        # create publisher for position estimate
        self.state_predict_pub = rospy.Publisher('EKF/predict', Twist, queue_size=10)
        self.state_updated_pub = rospy.Publisher('EKF/updated', Twist, queue_size=10)


        # set/initialize persistent variables
        self.enc_init = [False, False, False]
        self.enc_prev = [0.0, 0.0, 0.0]
        self.enc_curr = [0.0, 0.0, 0.0]
        self.state_predict = Twist()
        self.state_updated = Twist()

        self.cov_predict = np.array([   [1, 0, 0, 0, 0, 0], \
                                        [0, 1, 0, 0, 0, 0], \
                                        [0, 0, 1, 0, 0, 0], \
                                        [0, 0, 0, 1, 0, 0], \
                                        [0, 0, 0, 0, 1, 0], \
                                        [0, 0, 0, 0, 0, 1]])

        self.cov_updated = np.array([   [1, 0, 0, 0, 0, 0], \
                                        [0, 1, 0, 0, 0, 0], \
                                        [0, 0, 1, 0, 0, 0], \
                                        [0, 0, 0, 1, 0, 0], \
                                        [0, 0, 0, 0, 1, 0], \
                                        [0, 0, 0, 0, 0, 1]])
        self.F = np.array([ [1.0, 0.0, 0.0, 0.1, 0.0, 0.0 ], \
                            [0.0, 1.0, 0.0, 0.0, 0.1, 0.0 ], \
                            [0.0, 0.0, 1.0, 0.0, 0.0, 0.1 ], \
                            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0 ], \
                            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0 ], \
                            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0 ]])

        self.H = np.array([ [1.0, 0.0, 0.0], \
                            [0.0, 1.0, 0.0], \
                            [0.0, 0.0, 1.0], \
                            [0.0, 0.0, 0.0], \
                            [0.0, 0.0, 0.0], \
                            [0.0, 0.0, 0.0]])

        self.K = np.array([ [1.0, 0.0, 0.0], \
                            [0.0, 1.0, 0.0], \
                            [0.0, 0.0, 1.0], \
                            [0.0, 0.0, 0.0], \
                            [0.0, 0.0, 0.0], \
                            [0.0, 0.0, 0.0]])

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


    def predict(self, msg):
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
        delta_x_w = delta_x*math.cos(delta_w/2 + self.state_predict.linear.z) - delta_y*math.sin(delta_w/2 + self.state_predict.linear.z)
        delta_y_w = delta_x*math.sin(delta_w/2 + self.state_predict.linear.z) + delta_y*math.cos(delta_w/2 + self.state_predict.linear.z)

        # Element-wise assign or else assigned by reference (very bad)
        self.enc_prev[0] = self.enc_curr[0]
        self.enc_prev[1] = self.enc_curr[1]
        self.enc_prev[2] = self.enc_curr[2]

        self.state_predict.linear.x = delta_x_w + self.state_predict.linear.x
        self.state_predict.linear.y = delta_y_w + self.state_predict.linear.y
        self.state_predict.linear.z = delta_w   + self.state_predict.linear.z

        self.state_predict.angular.x = delta_x_w / (self.time_curr - self.time_prev)
        self.state_predict.angular.y = delta_y_w / (self.time_curr - self.time_prev)
        self.state_predict.angular.z = delta_w   / (self.time_curr - self.time_prev)

        self.time_curr = self.time_prev

        self.cov_predict = np.matmul(self.F, self.cov_predict) + np.matmul(self.cov_predict, np.transpose(self.F))


    # def update(self, msg):
    #     self.K =  np.matmul(self.cov_predict, np.matmul(np.transpose(self.H), np.linalg.inv(np.matmul(self.H , np.matmul(self.cov_predict , np.transpose(self.H))))))



        # self.state_updated = self.state_updated
        # self.cov_updated = self.cov_predict + np.matmul(self.K, )
        # pass

    def run(self):
        rate = rospy.Rate(10) # 10 Hz

        while(not rospy.is_shutdown()):

            self.state_predict_pub.publish( self.state_predict )

            rate.sleep()


if __name__ == '__main__':
    EKF_omnicone = EKF_omnicone()
    EKF_omnicone.run()
