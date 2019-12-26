#!/usr/bin/env python

import rospy
import math
import open_base
from std_msgs.msg import Float64
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

        self.K_err = 0.5


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
        for i in range(len(self.Waypoints.poses)-1):
            distance = math.sqrt((self.Waypoints.poses[i].pose.position.x - msg.linear.x)**2 + (self.Waypoints.poses[i].pose.position.x - msg.linear.x)**2)
            if distance < min:
                min = distance
                mindex = i 
        
        x0 = msg.linear.x
        y0 = msg.linear.y
        x1 = self.Waypoints.poses[mindex  ].pose.position.x
        x2 = self.Waypoints.poses[mindex+1].pose.position.x
        y1 = self.Waypoints.poses[mindex  ].pose.position.y
        y2 = self.Waypoints.poses[mindex+1].pose.position.y

        a = y1 - y2
        b = x2 - x1 
        c = -a*x1 - b*y1 
        
        x3 = (b*(b*x0-a*y0) - a*c) / (a**2 + b**2)
        y3 = (a*(a*y0-b*x0) - b*c) / (a**2 + b**2)

        dx_path = x2 - x1
        dy_path = y2 - y1

        path_norm = math.sqrt( dx_path ** 2 + dy_path ** 2 )
        Vx_path = dx_path / path_norm
        Vy_path = dy_path / path_norm

        Vx_err  = self.K_err * (x3 - x0)
        Vy_err  = self.K_err * (y3 - y0)        

        V_world  = math.sqrt((Vx_path + Vx_err)**2 + (Vy_path + Vy_err)**2)
        Vx_world = (Vx_path + Vx_err) / V_world
        Vy_world = (Vy_path + Vy_err) / V_world

        Vx = Vx_world * math.cos(msg.linear.z) - Vy_world * math.sin(msg.linear.z)
        Vy = Vx_world * math.sin(msg.linear.z) + Vy_world * math.cos(msg.linear.z)

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
