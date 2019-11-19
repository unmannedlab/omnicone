#!/usr/bin/env python

import rospy
import open_base
import math

from geometry_msgs.msg import Pose2D, Vector3

from math import pi
from open_base.srv import KinematicsInverse, KinematicsForward

import serial


class UBXCommander:

    def __init__(self):

        rospy.init_node('UBX_cmd')

        # Paramters

        self.lon_home = -96.3550552
        self.lat_home =  30.6239174

        self.circumference = pi * 0.101                         # meters
        self.linear_to_rot = 2 * pi / self.circumference        # rad / m
        self.command_timeout = 1.0                              # sec
        self.rpm = 30.0                                         # rev / minute
        self.circumference = 3.14159 * 0.101                    # meters
        self.linear_speed = self.rpm / 60 * self.circumference  # m/s

        # create the publisher for the goal velocity
        self.vel_pub = rospy.Publisher('robot_vel_goal', Pose2D, queue_size=10)

        self.ubx_gui_cmd      = rospy.Subscriber('/UBX/gui_cmd',        Pose2D,  self.UpdateCmd)
        self.ubx_hpposllh_sub = rospy.Subscriber('/UBX/hpposllh/llh',   Vector3, self.UpdatePos)
        self.ubx_relpos2D_sub = rospy.Subscriber('/UBX/relpos2D/pos',   Pose2D,  self.UpdateHdg)

        self.cmd_vel  = Pose2D()
        self.lon = 0
        self.lat = 0
        self.hdg = 0
        self.Xset = 0
        self.Yset = 0


    def UpdateCmd(self,msg):
        self.Xset = msg.x
        self.Yset = msg.y
        pass


    def UpdateHdg(self,msg):
        self.hdg = msg.theta


    def UpdatePos(self,msg):
        self.lon = msg.x
        self.lat = msg.y

        a = 6378137.0
        b = 6356752.3142

        R = math.sqrt(( math.pow( math.pow(a,2) * math.cos(self.lat) ,2)+ \
                        math.pow( math.pow(b,2) * math.sin(self.lat) ,2))/ \
                       (math.pow( a * math.cos(self.lat) ,2)+ \
                        math.pow( b * math.sin(self.lat) ,2))) + msg.z


        dlon = self.lon_home - self.lon             # degrees
        dlat = self.lat_home - self.lat             # degrees
        dE = R * math.radians(dlon) - self.Xset     # meters
        dN = R * math.radians(dlat) - self.Yset     # meters
        dT = 0 # (180 - self.hdg)

        print (dE)
        print (dN)
        print (dT)
        print ("")

        distance = ( dE ** 2 + dN ** 2) ** 0.5      # meters

        self.cmd_vel.x = -(dE * math.cos(dT) - dN * math.sin(dT)) / max(distance,1) * self.linear_speed
        self.cmd_vel.y = -(dE * math.sin(dT) + dN * math.cos(dT)) / max(distance,1) * self.linear_speed
        # self.cmd_vel.theta  = dT / max(abs(dT),20) * self.linear_speed / 3


    def run(self):
        rate = rospy.Rate(10) # 10 Hz

        while(not rospy.is_shutdown()):
            self.vel_pub.publish( self.cmd_vel)
            rate.sleep()


if __name__ == '__main__':
    UBX_command = UBXCommander()
    UBX_command.run()
