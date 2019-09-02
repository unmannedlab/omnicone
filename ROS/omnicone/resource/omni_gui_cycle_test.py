#!/usr/bin/env python

import sys, rospy
from math import pi
from geometry_msgs.msg import Pose2D
from PyQt5.QtCore import QObject, pyqtSlot
from PyQt5.QtWidgets import QApplication, QMainWindow
from omni_gui_ui import Ui_MainWindow

class OmniGui(QObject):

    def __init__(self):
        super(OmniGui, self).__init__()

        rospy.init_node('omni_gui')
        self.pub = rospy.Publisher('robot_vel_goal', Pose2D, queue_size=10)

	self.totalTime = rospy.get_param('~total_time', -1)
	self.numCycles = rospy.get_param('~num_cycles', 1)

        self.app = QApplication(sys.argv)
        self.window = QMainWindow()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self.window)

        self.currentX = 0.0
        self.currentY = 0.0

        self.linearSpeed = 4*pi*0.0254
        self.rpm = 60

	self.start_time = None

        self.ui.button0.released.connect(self.setDistance)
        self.ui.button1.released.connect(self.setDistance)
        self.ui.button2.released.connect(self.setDistance)
        self.ui.button3.released.connect(self.setDistance)
        self.ui.button4.released.connect(self.setDistance)
        self.ui.button5.released.connect(self.setDistance)
        self.ui.button6.released.connect(self.setDistance)
        self.ui.button7.released.connect(self.setDistance)
        self.ui.button8.released.connect(self.setDistance)
        self.ui.button9.released.connect(self.setDistance)

        self.ui.signButton.released.connect(self.switchSign)

        self.ui.resetButton.released.connect(self.resetDistance)

        self.ui.goButton.released.connect(self.move)

    def setDistance(self):
        number = float(self.sender().text()) * 0.1

        if self.ui.xSelect.isChecked():
            self.currentX *= 10
            self.currentX += number
            self.updateX()
        else:
            self.currentY *= 10
            self.currentY += number
            self.updateY()

    def switchSign(self):
        if self.ui.xSelect.isChecked():
            self.currentX *= -1
            self.updateX()
        else:
            self.currentY *= -1
            self.updateY()

    def updateX(self):
        self.ui.xDisplay.setText(str(self.currentX))

    def updateY(self):
        self.ui.yDisplay.setText(str(self.currentY))

    def resetDistance(self):
        self.currentX = 0.0
        self.currentY = 0.0

        self.updateX()
        self.updateY()

    def getDuration(self):
	return rospy.get_time() - self.start_time

    def move(self):

        msg = Pose2D()

        distance = (self.currentX ** 2 + self.currentY ** 2) ** 0.5

        time = distance/self.linearSpeed

	if self.totalTime == -1 or self.totalTime - 2 * time < 0:
	    wait_time = 4.0
	else:
	    wait_time = self.totalTime - 2 * time

        rate = rospy.Rate(10)

        for i in range(0, self.numCycles):
            self.start_time = rospy.get_time()
            while (not rospy.is_shutdown()):
                if (self.getDuration() > 2.0 and self.getDuration() < 2.0 + time):
                    msg.x = self.currentX / time
                    msg.y = self.currentY / time
                    msg.theta = 0.0

                elif (self.getDuration() > 2.0 + wait_time + time and self.getDuration() < 2.0 + wait_time + 2*time):
                    msg.x = -self.currentX / time
                    msg.y = -self.currentY / time
                    msg.theta = 0.0

                elif self.getDuration() > 6.0 + wait_time + 2*time:
                    break

                else:
                    msg.x = 0.0
                    msg.y = 0.0
                    msg.theta = 0.0

                self.pub.publish(msg)
                rate.sleep()

    def run(self):
        self.window.show()
        sys.exit(self.app.exec_())

if __name__ == '__main__':
    gui = OmniGui()
    gui.run()
