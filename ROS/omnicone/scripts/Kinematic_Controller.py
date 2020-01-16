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
        # Kinematic Controller object initialization
        # Description:
        #     Function inializes the Kinematic Controller object. Subscribes 
        #     to state and path and publsihes commands. Initializes persistent 
        #     variables.   

        rospy.init_node('Kinematic_Controller')

        # Paramters
        self.circumference = pi * 0.101                     # meters
        self.linear_to_rot = 2 * pi / self.circumference    # rad / m
        self.command_timeout = 1.0                          # sec

        # Create the command publishers
        self.left_pub  = rospy.Publisher('left_joint_velocity_controller/command' , Float64, queue_size=10)
        self.back_pub  = rospy.Publisher('back_joint_velocity_controller/command' , Float64, queue_size=10)
        self.right_pub = rospy.Publisher('right_joint_velocity_controller/command', Float64, queue_size=10)

        # Create the state and path subscribers
        self.state_sub = rospy.Subscriber('/EKF/state', Twist, self.updateVels)
        self.path_sub  = rospy.Subscriber('Waypoints',  Path,  self.updatePath)

        # Create the caller to the inverse kinematics server
        self.ik_service_client = rospy.ServiceProxy('/kinematics_inverse_world', KinematicsInverse)

        # Initialize persistent variables
        self.cmd_vel  = [0.0, 0.0, 0.0]
        self.last_cmd_time = rospy.get_time()
        self.Waypoints = Path()
        self.path_index = -1

        # Kinematic Controller Values
        self.K_err = 1.0
        self.desired_speed = 0.5


    def updatePath(self, msg):
        # Update Desired Path
        # Description:
        #     Function updates the internal desired path for the robot.

        self.Waypoints = msg


    def updateVels(self, msg):
        # Update Command Velocities
        # Description:
        #     Function uses EKF state information and desired path to publish
        #     command velocities using a kinematic controller. The nearest point 
        #     on the path is found and control and error velocities are 
        #     controlled to that section of the path. 

        # Look for closest point and update global velocity goal
        dist_min = 10e9

        if self.path_index == -1:
            # If first search of path for closest pose, checks all path poses
            for i in range(len(self.Waypoints.poses)-1):
                distance = math.sqrt((self.Waypoints.poses[i].pose.position.x - msg.linear.x)**2 + \
                                     (self.Waypoints.poses[i].pose.position.y - msg.linear.y)**2)
                if distance < dist_min:
                    dist_min = distance
                    self.path_index = i
            
        else:
            # Otherwise, only checks nearest 50 poses for closest path pose
            for i in range(max(0,self.path_index-25), \
                           min(self.path_index+25,len(self.Waypoints.poses)-1)):
                distance = math.sqrt((self.Waypoints.poses[i].pose.position.x - msg.linear.x)**2 + \
                                     (self.Waypoints.poses[i].pose.position.y - msg.linear.y)**2)
                if distance < dist_min:
                    dist_min = distance
                    self.path_index = i

        # P0 - Current state position
        x0 = msg.linear.x
        y0 = msg.linear.y
        # P1 - Closest path pose
        x1 = self.Waypoints.poses[self.path_index].pose.position.x
        y1 = self.Waypoints.poses[self.path_index].pose.position.y
        # P2 - Next path pose
        x2 = self.Waypoints.poses[self.path_index+1].pose.position.x
        y2 = self.Waypoints.poses[self.path_index+1].pose.position.y

        if self.path_index == len(self.Waypoints.poses)-2:
            # If closest to last point, control to final point and stop 
            dx_world = x2 - x0
            dy_world = y2 - y0
            distance = math.sqrt(dx_world**2 + dy_world**2)
            
            # Normalizes velocities to error from final point
            #   Ramps down speed when less than 1 m away
            Vx = -(dx_world * math.cos(msg.linear.z*pi/180) -   \
                   dy_world * math.sin(msg.linear.z*pi/180)) /  \
                       max(distance,1) * self.desired_speed

            Vy = -(dx_world * math.sin(msg.linear.z*pi/180) +   \
                   dy_world * math.cos(msg.linear.z*pi/180)) /  \
                       max(distance,1) * self.desired_speed

            # Transform to joint velocities using inverse kinematics
            goal = Pose2D(Vx,Vy,0)
            resp = self.ik_service_client(goal)

            # Convert to rotational speeds
            self.cmd_vel[0] = -resp.output.v_left  * self.linear_to_rot
            self.cmd_vel[1] = -resp.output.v_back  * self.linear_to_rot
            self.cmd_vel[2] = -resp.output.v_right * self.linear_to_rot


        else:
            # Calculate Line equation between P1 and P2 in the following form:
            #     a*x + b*y + c = 0
            a = y1 - y2
            b = x2 - x1 
            c = -a*x1 - b*y1 
            
            # P3 - Closest point to P0 on line betweeen P1 and P2
            x3 = (b*(b*x0-a*y0) - a*c) / (a**2 + b**2)
            y3 = (a*(a*y0-b*x0) - b*c) / (a**2 + b**2)

            # Desired path velocity 
            dx_path = x2 - x1
            dy_path = y2 - y1

            # Normalized path velocity
            path_norm = math.sqrt( dx_path ** 2 + dy_path ** 2 )
            Vx_path = dx_path / path_norm
            Vy_path = dy_path / path_norm

            # Error velocity
            Vx_err  = self.K_err * (x3 - x0)
            Vy_err  = self.K_err * (y3 - y0)        

            # Normalize desired and error correction velocities
            V_world  = math.sqrt((Vx_path + Vx_err)**2 + (Vy_path + Vy_err)**2)
            Vx_world = (Vx_path + Vx_err) / V_world * self.desired_speed
            Vy_world = (Vy_path + Vy_err) / V_world * self.desired_speed

            # Transform global control velocity to local control velocity
            Vx = -Vx_world * math.cos(msg.linear.z*pi/180) + \
                  Vy_world * math.sin(msg.linear.z*pi/180)
            Vy = -Vx_world * math.sin(msg.linear.z*pi/180) - \
                  Vy_world * math.cos(msg.linear.z*pi/180)

            # Transform to joint velocities using inverse kinematics
            goal = Pose2D(Vx,Vy,0)
            resp = self.ik_service_client(goal)

            # Convert to rotational speeds
            self.cmd_vel[0] = -resp.output.v_left  * self.linear_to_rot
            self.cmd_vel[1] = -resp.output.v_back  * self.linear_to_rot
            self.cmd_vel[2] = -resp.output.v_right * self.linear_to_rot

        self.last_cmd_time = rospy.get_time()

    def run(self):
        # Kinematic Controller Publisher
        # Description:
        #     Function will continually publish control velocities regardless of 
        #     EKF callback rate based on latest update. Will stop the robot if
        #     EKF topic is lost for more than 1 second. Publishes commands as 
        #     individual joint commands.

        rate = rospy.Rate(10) # 1 Hz

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
