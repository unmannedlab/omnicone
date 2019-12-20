#!/usr/bin/env python

import rospy
import math
import open_base
# from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Path

from math import pi
from open_base.srv import KinematicsInverse


class Kinematic_Controller:

    def __init__(self):
        # Title
        # Description:
        #     Function does

        rospy.init_node('Kinematic_Controller')

        # Paramters
        self.circumference = pi * 0.101                     # meters
        self.linear_to_rot = 2 * pi / self.circumference    # rad / m
        self.command_timeout = 1.0                          # sec

        # create the publishers
        self.left_pub  = rospy.Publisher('left_joint_velocity_controller/command' , Float64, queue_size=10)
        self.back_pub  = rospy.Publisher('back_joint_velocity_controller/command' , Float64, queue_size=10)
        self.right_pub = rospy.Publisher('right_joint_velocity_controller/command', Float64, queue_size=10)

        # create the subscriber
        self.state_sub = rospy.Subscriber('/EKF/state', Twist, self.updateVels)
        self.path_sub  = rospy.Subscriber('Waypoints',  Path,  self.updatePath)


        # create the caller to the inverse kinematics server
        self.ik_service_client = rospy.ServiceProxy('/kinematics_inverse_world', KinematicsInverse)

        # initialize persistent variables
        self.cmd_vel  = [0.0, 0.0, 0.0]
        self.last_cmd_time = rospy.get_time()
        self.Waypoints = Path()


    def updatePath(self, msg):
        # Title
        # Description:
        #     Function does

        self.Waypoints = msg


    def updateVels(self, msg):
        # Title
        # Description:
        #     Function does

        # Look for closest point and update global velocity goal
        min = 10^9
        mindex = 0
        for Waypoint in self.Waypoints:
            if sqrt((Waypoint.pose.position.x - msg.linear.x)**2 + (Waypoint.pose.position.x - msg.linear.x)**2) < min

        Vx = 0
        Vy = 0

        goal = Pose2D(Vx,Vy,0)
        resp = self.ik_service_client(goal)

        self.cmd_vel[0] = -resp.output.v_left  * self.linear_to_rot
        self.cmd_vel[1] = -resp.output.v_back  * self.linear_to_rot
        self.cmd_vel[2] = -resp.output.v_right * self.linear_to_rot

        self.last_cmd_time = rospy.get_time()

    def run(self):
        # Title
        # Description:
        #     Function does

        rate = rospy.Rate(1) # 1 Hz

        while(not rospy.is_shutdown()):
            if ( rospy.get_time() - self.last_cmd_time > self.command_timeout):
                self.cmd_vel = [0.0, 0.0, 0.0]

            self.left_pub.publish( self.cmd_vel[0])
            self.back_pub.publish( self.cmd_vel[1])
            self.right_pub.publish(self.cmd_vel[2])
            rate.sleep()


if __name__ == '__main__':
    K_Control = Kinematic_Controller()
    K_Control.run()
