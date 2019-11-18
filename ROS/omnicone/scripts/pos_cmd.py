#!/usr/bin/env python

import rospy
import open_base
import math
from std_msgs.msg import Float32, Float64, Int32
from geometry_msgs.msg import Pose2D, Point, PoseStamped

from math import pi
from open_base.srv import KinematicsInverse, KinematicsForward

class PosCommander:

    def __init__(self):
        # Velocity Commander Initialization
        # Description:
        #     Function initializes the velocity commander by creating the node,
        #     subscribers, and publishers. Additionally, the parameters and
        #     persistent variables are set/initialized to zero.

        rospy.init_node('pos_cmd')

        # Paramters
        self.circumference = pi * 0.101                     # meters
        self.linear_to_rot = 2 * pi / self.circumference    # rad / m
        self.command_timeout = 1.0                          # sec

        # create the publisher for each individual
        self.left_pub  = rospy.Publisher('left_joint_velocity_controller/command' , Float64, queue_size=10)
        self.back_pub  = rospy.Publisher('back_joint_velocity_controller/command' , Float64, queue_size=10)
        self.right_pub = rospy.Publisher('right_joint_velocity_controller/command', Float64, queue_size=10)

        self.pos_goal_sub = rospy.Subscriber('/robot_pos_goal', Pose2D, self.UpdateVels)
        self.pos_goal_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.UpdatePos)

        self.cmd_vel  = [0.0, 0.0, 0.0]
        self.pos_x = 0
        self.pos_y = 0


    def UpdatePos(self,msg):
        self.pos_x = msg.pose.point.x
        self.pos_y = msg.pose.point.y


    def UpdateVels(self,msg):

        distance = ((msg.x - self.currentX) ** 2 + (msg.y - self.currentY) ** 2) ** 0.5 # meters

        self.cmd_vel[0] = (msg.x - self.currentX) / distance * self.linear_speed
        self.cmd_vel[1] = (msg.y - self.currentY) / distance * self.linear_speed
        self.cmd_vel[2] = 0.0


    def run(self):
        rate = rospy.Rate(10) # 10 Hz

        while(not rospy.is_shutdown()):
            if ( rospy.get_time() - self.last_command > self.command_timeout):
                self.cmd_vel = [0.0, 0.0, 0.0]

            self.left_pub.publish( self.cmd_vel[0])
            self.back_pub.publish( self.cmd_vel[1])
            self.right_pub.publish(self.cmd_vel[2])
            # self.pos_pub.publish(self.d_pos_est)
            rate.sleep()


if __name__ == '__main__':
    pos_command = PosCommander()
    pos_command.run()
