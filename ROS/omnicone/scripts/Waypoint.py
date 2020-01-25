#!/usr/bin/env python

import os 
import rospy
import rospkg
import math
import open_base
# from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Pose2D, PoseStamped
from nav_msgs.msg import Path

from math import pi


class Waypoint_Publisher:

    def __init__(self, filename):
        # Title
        # Description:
        #     Function does

        rospy.init_node('Waypoint')

        # create the publisher
        self.path_pub  = rospy.Publisher('Waypoints' , Path, latch=True, queue_size=1)

        # filename = rospy.get_param('/WaypointFilePath')
        self.LoadWaypoints(filename)


    def LoadWaypoints(self, filename):
        # Title
        # Description:
        #     Function does
        #     Appropriated from Amir Darwesh (2019)

        self.waypoints = Path()
        self.waypoints.header.stamp = rospy.Time.now()
        self.waypoints.header.frame_id = "world"
        cx,cy = [], []

        rospack = rospkg.RosPack()
        with open( os.path.join(rospack.get_path('omnicone'),'scripts', filename) ) as f:
            for line in f:
                rx, ry = map(float,line.split(','))
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
            self.waypoints.poses.append(cpose)

    def run(self):
        # Title
        # Description:
        #     Function does

        rate = rospy.Rate(1) # 1 Hz

        while(not rospy.is_shutdown()):

            self.path_pub.publish(self.waypoints)
            rate.sleep()


if __name__ == '__main__':
    WayPub = Waypoint_Publisher("SquarePath.csv")
    WayPub.run()
