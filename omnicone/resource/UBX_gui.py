#!/usr/bin/env python

import os, sys, rospkg

import rospy
from geometry_msgs.msg import Pose2D

from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QObject


class OmniGui(QObject):

    def __init__(self):
        super(OmniGui, self).__init__()

        rospy.init_node('gui_cmd', disable_signals=True)
        self.pub = rospy.Publisher('UBX/gui_cmd', Pose2D, queue_size=10)

        self.app = QtWidgets.QApplication(sys.argv)

        rospack = rospkg.RosPack()
        self.window = uic.loadUi(rospack.get_path('omnicone') + '/resource/cmd_gui.ui')

        self.reset_program()

        self.rpm = 50.0                                         # rev / minute
        self.circumference = 3.14159 * 0.101                    # meters
        self.linear_speed = self.rpm / 60 * self.circumference  # m/s

        self.window.pushButton_0.released.connect(self.setDistance)
        self.window.pushButton_1.released.connect(self.setDistance)
        self.window.pushButton_2.released.connect(self.setDistance)
        self.window.pushButton_3.released.connect(self.setDistance)
        self.window.pushButton_4.released.connect(self.setDistance)
        self.window.pushButton_5.released.connect(self.setDistance)
        self.window.pushButton_6.released.connect(self.setDistance)
        self.window.pushButton_7.released.connect(self.setDistance)
        self.window.pushButton_8.released.connect(self.setDistance)
        self.window.pushButton_9.released.connect(self.setDistance)
        self.window.pushButton_pt.released.connect(self.setDecimal)
        self.window.pushButton_pm.released.connect(self.switchSign)

        self.window.pushButton_start.released.connect(self.move)
        self.window.pushButton_reset.released.connect(self.reset_program)
        self.window.pushButton_end.released.connect(self.end_program)

        self.window.show()
        sys.exit(self.app.exec_())


    def setDistance(self):
        if self.window.radioButton_x.isChecked():
            if self.X_dec_switch:
                self.currentX += float(self.sender().text()) * 10 ** self.X_dec_place
                self.X_dec_place -= 1
            else:
                self.currentX = self.currentX * 10 + float(self.sender().text())

            self.updateX()
        else:
            if self.Y_dec_switch:
                self.currentY += float(self.sender().text()) * 10 ** self.Y_dec_place
                self.Y_dec_place -= 1
            else:
                self.currentY = self.currentY * 10 + float(self.sender().text())

            self.updateY()


    def setDecimal(self):
        if self.window.radioButton_x.isChecked():
            self.X_dec_switch = True
        else:
            self.Y_dec_switch = True


    def switchSign(self):
        if self.window.radioButton_x.isChecked():
            self.X_sign *= -1
            self.updateX()
        else:
            self.Y_sign *= -1
            self.updateY()


    def move(self):
        start_time = rospy.get_rostime()
        msg = Pose2D()

        distance = (self.currentX ** 2 + self.currentY ** 2) ** 0.5 # meters

        time = distance / self.linear_speed                         # seconds

        rate = rospy.Rate(10)
        self.run_switch = True
        while (self.run_switch):
            self.app.processEvents()
            if (rospy.get_rostime() - start_time > rospy.Duration.from_sec(1.0) and
            rospy.get_rostime() - start_time < rospy.Duration.from_sec(5.0 + time)):
                msg.x = self.currentX
                msg.y = self.currentY
                msg.theta = 0.0
            else:
                msg.x = 0.0
                msg.y = 0.0
                msg.theta = 0.0

            self.pub.publish(msg)
            rate.sleep()


    def reset_program(self):
        self.currentX = 0.0
        self.currentY = 0.0
        self.X_dec_switch = False
        self.Y_dec_switch = False
        self.run_switch = False
        self.X_dec_place = -1
        self.Y_dec_place = -1
        self.X_sign = 1
        self.Y_sign = 1
        self.window.radioButton_x.setChecked(True)
        self.updateX()
        self.updateY()


    def end_program(self):
        print('GUI Exit')
        os.system('rosnode kill -a')
        sys.exit(0)


    def updateX(self):
        self.window.lcdNumber_x_entry.display(str(round(self.X_sign*self.currentX,2)))


    def updateY(self):
        self.window.lcdNumber_y_entry.display(str(round(self.Y_sign*self.currentY,2)))

if __name__ == '__main__':
    gui = OmniGui()
