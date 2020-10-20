#!/usr/bin/env python3

import os, sys, rospkg

import rospy
from std_msgs.msg import Bool, Int8
from geometry_msgs.msg import Pose2D
from omnicone_msgs.msg import ConeState

from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QObject


def mode_to_str(mode, proxy):
    if proxy:
        return " Waiting"
    elif(mode == 0):
        return " Initialized"
    elif(mode == 1):
        return " Deploying"
    elif(mode == 2):
        return " Idle"
    elif(mode == 3):
        return " Returning"
    elif(mode == 4):
        return " Complete"
    else:
        return " ERROR"


class cone_cmd(QObject):

    def __init__(self):
        super(cone_cmd, self).__init__()

        # GUI Message Publishers
        rospy.init_node('cone_cmd', disable_signals=True)
        self.kill_pub = rospy.Publisher('gui/kill', Bool, queue_size=10)
        self.mode_pub = rospy.Publisher('gui/mode', Int8, queue_size=10)

        # GUI Subscribers
        self.cone1_sub = rospy.Subscriber('recv/ConeState1', ConeState, self.update_ConeState_1)
        self.cone2_sub = rospy.Subscriber('recv/ConeState2', ConeState, self.update_ConeState_2)
        self.cone2_sub = rospy.Subscriber('recv/ConeState3', ConeState, self.update_ConeState_3)

        self.app = QtWidgets.QApplication(sys.argv)

        rospack = rospkg.RosPack()
        self.window = uic.loadUi(rospack.get_path('omnicone') + '/resource/cone_cmd.ui')

        # Initialize Persistent Variables
        self.cmd_kill = False
        self.cmd_mode = 0

        self.reset_gui()
        
        self.window.pushButton_deploy.released.connect(self.cmd_deploy)
        self.window.pushButton_return.released.connect(self.cmd_return)
        self.window.pushButton_abort.released.connect( self.cmd_abort)
        self.window.pushButton_exit.released.connect( self.end_program)

        self.window.show()
        sys.exit(self.app.exec_())
        
    # Cone 1 Callback
    def update_ConeState_1(self, msg):
        self.window.label_cone1.setText(mode_to_str(msg.mode, msg.proximity_alarm))


    # Cone 2 Callback
    def update_ConeState_2(self, msg):
        self.window.label_cone2.setText(mode_to_str(msg.mode, msg.proximity_alarm))
        

    # Cone 3 Callback
    def update_ConeState_3(self, msg):
        self.window.label_cone3.setText(mode_to_str(msg.mode, msg.proximity_alarm))

    # Deploy Button
    def cmd_deploy(self):
        self.cmd_mode = 1
        self.window.label_cmd.setText(" 1: Deploy")
        self.mode_pub.publish(self.cmd_mode)

    # Return Button
    def cmd_return(self):
        self.cmd_mode = 3
        self.window.label_cmd.setText(" 1: Return")
        self.mode_pub.publish(self.cmd_mode)

    # Abort Button
    def cmd_abort(self):
        self.cmd_kill = True 
        self.window.label_kill.setText(" True")
        self.kill_pub.publish(self.cmd_kill)

    # Reset Variables
    def reset_gui(self):
        self.window.label_cone1.setText(" Offline")
        self.window.label_cone2.setText(" Offline")
        self.window.label_cone3.setText(" Offline")
        self.window.label_kill.setText(" False")

    # Node shutdown via gui
    def end_program(self):
        print('GUI Exit')
        self.mode_pub.publish(4)
        self.kill_pub.publish(self.cmd_kill)
        os.system('rosnode kill -a')
        sys.exit(0)


if __name__ == '__main__':
    gui = cone_cmd()
