#!/usr/bin/env python

import rospy
import open_base
import math
import numpy as np
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist, Pose2D, Vector3

from math import pi
class EKF_omnicone:


    def __init__(self):

        rospy.init_node('EKF')

        # Paramters
        self.circumference = pi * 0.101                     # meters
        self.linear_to_rot = 2 * pi / self.circumference    # rad / m
        self.command_timeout = 1.0                          # sec

        self.lon_home = -96.345611
        self.lat_home =  30.6128444


        # create the subscriber for the encoder_pulses_per_revolution
        self.left_enc_sub  = rospy.Subscriber( '/left_joint_velocity_controller/absolute_encoder_count', Int32, self.updateEnc_left)
        self.back_enc_sub  = rospy.Subscriber( '/back_joint_velocity_controller/absolute_encoder_count', Int32, self.updateEnc_back)
        self.right_enc_sub = rospy.Subscriber('/right_joint_velocity_controller/absolute_encoder_count', Int32, self.predict)
        self.UBX_rel_pos_sub = rospy.Subscriber('/UBX/relpos2D/err', Pose2D  , self.update_relpos2d_err)
        self.UBX_rel_err_sub = rospy.Subscriber('/UBX/relpos2D/pos', Pose2D  , self.update_relpos2d_pos)
        self.UBX_llh_llh_sub = rospy.Subscriber('/UBX/hpposllh/err', Vector3 , self.update_hpposllh_err)
        self.UBX_llh_err_sub = rospy.Subscriber('/UBX/hpposllh/llh', Vector3 , self.update)

        # create publisher for position estimate
        self.state_pub = rospy.Publisher('EKF/state', Twist, queue_size=10)
        self.state_twist = Twist()

        # set/initialize persistent variables
        self.enc_init = [False, False, False]
        self.enc_prev = [0.0, 0.0, 0.0]
        self.enc_curr = [0.0, 0.0, 0.0]

        self.UBX_rel_pos = [0.0, 0.0, 0.0]
        self.UBX_llh_llh = [0.0, 0.0, 0.0]

        self.state = np.array([[0.0],[0.0],[180.0],[0.0],[0.0],[0.0]])

        self.Process_Q = np.array([ [ 4.74957585660, -0.0460417908,	-0.8333033435,	 0.0024956610,	 0.2439671528,	0.0667347090],\
                                    [-0.04604179080,  4.5408676761,	 0.8832843155,	-0.2447392906,	 0.2439671528,	0.1076589568],\
                                    [-0.83330334350,  0.8832843155,	 4.1847115351,	-0.0488464530,	-0.0898108461,	0.8532518865],\
                                    [ 0.00249566100, -0.2447392906,	-0.0488464530,	 0.0294990109,	-0.0003074584, -0.0501564979],\
                                    [ 0.24396715208,  0.2439671528,	-0.0898108461,	-0.0003074584,	 0.0246242047, 	0.0222673897],\
                                    [ 0.06673470900,  0.1076589568,	 0.8532518865,	-0.0501564979,	 0.0222673897,	8.170778926]])

        self.state_cov = self.Process_Q

        self.Transition_F = np.array([  [1.0, 0.0, 0.0, 0.2, 0.0, 0.0 ], \
                                        [0.0, 1.0, 0.0, 0.0, 0.2, 0.0 ], \
                                        [0.0, 0.0, 1.0, 0.0, 0.0, 0.2 ], \
                                        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0 ], \
                                        [0.0, 0.0, 0.0, 0.0, 1.0, 0.0 ], \
                                        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0 ]])

        self.Observation_H = np.array([ [1.0, 0.0, 0.0, 0.0, 0.0, 0.0], \
                                        [0.0, 1.0, 0.0, 0.0, 0.0, 0.0], \
                                        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]])

        self.Kalman_gain_K = np.array([ [1.0, 0.0, 0.0], \
                                        [0.0, 1.0, 0.0], \
                                        [0.0, 0.0, 1.0], \
                                        [0.0, 0.0, 0.0], \
                                        [0.0, 0.0, 0.0], \
                                        [0.0, 0.0, 0.0]])

        self.Observation_cov_R = np.array([ [1.0, 0.0, 0.0], \
                                            [0.0, 1.0, 0.0], \
                                            [0.0, 0.0, 1.0]])

        self.innovation_res_y = np.array([  [0],[0],[0]])

        self.innovation_cov_S = np.array([  [1.0, 0.0, 0.0], \
                                            [0.0, 1.0, 0.0], \
                                            [0.0, 0.0, 1.0]])


    def update_relpos2d_err(self, msg):
        # UBX publisher RelPosNED Position Callback
        # Description:
        #     Function updates the theta component of the Observation Covariance
        #     Matrix [R]. This value is estimated by the UBX F9P module.
        self.Observation_cov_R[2,2] = msg.theta


    def update_relpos2d_pos(self, msg):
        # UBX publisher RelPosNED Position Callback
        # Description:
        #     Function updates the theta component of the internal state for use
        #     in the Update() funtion
        self.UBX_rel_pos[2] = msg.theta


    def update_hpposllh_err(self, msg):
        # UBX publisher HPPosLLH Error Callback
        # Description:
        #     Function updates the x and y components of the Observation
        #     Covariance Matrix [R]. This value is estimated by the UBX F9P
        #     module.
        self.Observation_cov_R[0,0] = msg.x
        self.Observation_cov_R[1,1] = msg.y


    def updateEnc_left(self, msg):
        # Left Motor Encoder Callback
        # Description
        #     Function updates the internal memory of the left encoder counter.
        #     If the variable has not been initialized, the first value is
        #     assigned to both the previous and current encoder value.
        self.enc_curr[0] = msg.data

        # Initialize Encoder Value
        if not self.enc_init[0]:
            self.enc_prev[0] = self.enc_curr[0]
            self.enc_init[0] = True


    def updateEnc_back(self, msg):
        # Back Motor Encoder Callback
        # Description
        #     Function updates the internal memory of the back encoder counter.
        #     If the variable has not been initialized, the first value is
        #     assigned to both the previous and current encoder value.
        self.enc_curr[1] = msg.data

        # Initialize Encoder Value
        if not self.enc_init[1]:
            self.enc_prev[1] = self.enc_curr[1]
            self.enc_init[1] = True


    def predict(self, msg):
        # Right Motor Encoder Callback
        # Description
        #     Function updates the internal memory of the right encoder counter.
        #     If the variable has not been initialized, the first value is
        #     assigned to both the previous and current encoder value.
        #     Additionally, function integrates the wheel rotations and uses the
        #     omni base forward kinematics to estimate a change in position. This
        #     estimation is used as the prediction step of the Kalman filter.

        self.enc_curr[2] = msg.data
        # Initialize Encoder Value
        if not self.enc_init[2]:
            self.enc_prev[2] = self.enc_curr[2]
            self.enc_init[2] = True
            self.time_prev = rospy.get_time()

        self.time_curr = rospy.get_time()

        # Determine Rotation Angle of Each Motor 
        # num encoder counts / (4 counts / cycle ) / ( 7 * 27 cycles / rev) (2pi rad / rev)
        theta_l = -float(self.enc_curr[0] - self.enc_prev[0]) / 4 / (7 * 27) * (2 * pi)
        theta_b = -float(self.enc_curr[1] - self.enc_prev[1]) / 4 / (7 * 27) * (2 * pi)
        theta_r = -float(self.enc_curr[2] - self.enc_prev[2]) / 4 / (7 * 27) * (2 * pi)

        # Convert Angular to Linear 
        # v_linear = omega * d / 2
        D1 = theta_l * 0.101 / 2
        D2 = theta_b * 0.101 / 2
        D3 = theta_r * 0.101 / 2

        # Forward Kinematics Transformation
        delta_x = (2*D2 - D1 - D3) / 3
        delta_y = math.sqrt(3) * (D3 - D1) / 3
        delta_w = (D1 + D2 + D3)/(3 * 0.1905)
	
        # Rotation to World Frame
        delta_x_w = -delta_x*math.cos(delta_w/2 + self.state[2]*pi/180) + \
                     delta_y*math.sin(delta_w/2 + self.state[2]*pi/180)
        delta_y_w = -delta_x*math.sin(delta_w/2 + self.state[2]*pi/180) - \
                     delta_y*math.cos(delta_w/2 + self.state[2]*pi/180)

        # Element-wise assign or else assigned by reference (very bad)
        self.enc_prev[0] = self.enc_curr[0]
        self.enc_prev[1] = self.enc_curr[1]
        self.enc_prev[2] = self.enc_curr[2]

        # Predict Position States
        self.state[0] = self.state[0] + delta_x_w  
        self.state[1] = self.state[1] + delta_y_w 
        self.state[2] = self.state[2] + delta_w   * 180 / pi

        # Predict Velocity States
        self.state[3] = delta_x_w / (self.time_curr - self.time_prev)
        self.state[4] = delta_y_w / (self.time_curr - self.time_prev)
        self.state[5] = delta_w   / (self.time_curr - self.time_prev) * 180 / pi
        
        self.time_prev = self.time_curr

        # P_k|k-1 = F_k * P_k-1|k-1 * F_k^T + Q_k
        self.state_cov = np.matmul(self.Transition_F, np.matmul(self.state_cov, np.transpose(self.Transition_F))) + self.Process_Q

    def update(self, msg):
        # UBX publisher HPPosLLH LLH Callback
        # Description:
        #     Function uses the ellipsoid model of Earth to calculate a 2D
        #     position of the vehicle. This position, in conjunction with the
        #     measurement of the vehicle heading is used in the update step in
        #     the Kalman filter.

        # Ellipsoidal model calculation of Earth
        a = 6378137.0
        b = 6356752.3142
        R = math.sqrt(( math.pow( math.pow(a,2) * math.cos(msg.y*pi/180), 2)+ \
                        math.pow( math.pow(b,2) * math.sin(msg.y*pi/180), 2))/ \
                       (math.pow( a * math.cos(msg.y*pi/180), 2)+ \
                        math.pow( b * math.sin(msg.y*pi/180), 2))) + msg.z

        # Transform LLH to 2D position
        dlon = msg.x - self.lon_home    # degrees
        dlat = msg.y - self.lat_home    # degrees
        dE = R * math.radians(dlon)     # meters
        dN = R * math.radians(dlat)     # meters
        dT = self.UBX_rel_pos[2]        # degrees

        # Measurement in matrix format
        measurement = np.array([[dE],[dN],[dT]])

        # y_k       = z_k - h(xhat_k|k-1)
        self.innovation_res_y = measurement - np.matmul(self.Observation_H, self.state)

        # S_k       = H_k * P_k|k-1 * H_k^T + R_kError
        self.innovation_cov_S = np.matmul(self.Observation_H, np.matmul(self.state_cov, np.transpose(self.Observation_H))) + self.Observation_cov_R

        # K_k       = P_k|k-1 * H_k^T * S_k^-1
        self.Kalman_gain_K = np.matmul(self.state_cov, np.matmul(np.transpose(self.Observation_H), np.linalg.inv(self.innovation_cov_S)))

        # xhat_k|k  = xhat_k|k-1 + K_k * y_k
        self.state = self.state + np.matmul(self.Kalman_gain_K, self.innovation_res_y)

        # P_k|k     = (I - K_k * H_k) * P_k|k-1
        self.state_cov = np.matmul((np.eye(6) - np.matmul(self.Kalman_gain_K, self.Observation_H)), self.state_cov)


    def run(self):
        # State Estimation Publisher
        # Description:
        #     Function transforms state estimate from Kalman filter into Twist
        #     format and publishes at 10 hz. Is not dependent on callbacks. 
        #     The twist message components are redefined as Pose2D for 
        #     the linear component and Pose2D's derivative as the 
        #     angular component. 

        rate = rospy.Rate(10) # 10 Hz
        while(not rospy.is_shutdown()):

            self.state_twist.linear.x  = self.state[0]
            self.state_twist.linear.y  = self.state[1]
            self.state_twist.linear.z  = self.state[2]
            self.state_twist.angular.x = self.state[3]
            self.state_twist.angular.y = self.state[4]
            self.state_twist.angular.z = self.state[5]
            self.state_pub.publish( self.state_twist )

            rate.sleep()


if __name__ == '__main__':
    EKF_omnicone = EKF_omnicone()
    EKF_omnicone.run()
