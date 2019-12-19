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

        # create the publishers
        self.left_pub  = rospy.Publisher('left_joint_velocity_controller/command' , Float64, queue_size=10)

        # create the caller to the inverse kinematics server
        self.ik_service_client = rospy.ServiceProxy('/kinematics_inverse_world', KinematicsInverse)

        filepath = rospy.get_param('/WaypointFilePath')
        self.LoadWaypoints("SquarePath.csv")


    def LoadWaypoints(self, filename):
        # Title
        # Description:
        #     Function does
        #     Appropriated from Amir Darwesh (2019)

        waypointPath = Path()
        waypointPath.header.stamp = rospy.Time.now()
        waypointPath.header.frame_id = "world"
        cx,cy = [], []
        with open(filename) as f:
            for line in f:

                ry, rx = map(float,line.split(','))
                cx.append(rx)
                cy.append(ry)

        #x, y, _, _, _ = CubicSplinePlanner.calc_spline_course(cx,cy,ds=1)
        x,y = cx,cy
        for i in range(len(x)):

            cpose = PoseStamped()
            cpose.header.stamp = rospy.Time.now()
            cpose.header.frame_id = "world"
            cpose.pose.position.x, cpose.pose.position.y = x[i],y[i]
            cpose.pose.position.z = 0.0
            cpose.pose.orientation.x = 0
            cpose.pose.orientation.y = 0
            cpose.pose.orientation.z = 0
            cpose.pose.orientation.w = 1
            waypointPath.poses.append(cpose)

    def run(self):
        # Title
        # Description:
        #     Function does

        rate = rospy.Rate(1) # 1 Hz

        while(not rospy.is_shutdown()):

            self.right_pub.publish(self.waypoints)
            rate.sleep()


if __name__ == '__main__':
    K_Control = Kinematic_Controller()
    K_Control.run()
