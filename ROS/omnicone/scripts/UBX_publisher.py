#!/usr/bin/env python
"""An example of how to use the light parser for parsing UBX messages from a serial device.
You will need to change the port name to that of the port you want to connect to. Also make sure that the baud rate is
correct and that the device has been setup to output the messages via UBX protocol to your desired port!
The serial package could easily be replaced with an alternative.
"""

import serial
import rospy

from geometry_msgs.msg import Pose2D, Vector3
from ubxtranslator.core import Parser
from ubxtranslator.predefined import NAV_CLS, ACK_CLS
from geometry_msgs.msg import Pose2D


class UBXNode:

    def __init__(self):

        rospy.init_node('UBX')

        # Paramters
        self.port = serial.Serial("/dev/ArduSimple", baudrate=9600, timeout=0.1)

        self.hpposllh_pub  = rospy.Publisher('UBX/hpposllh', Vector3, queue_size=10)
        self.relpos2D_pub  = rospy.Publisher('UBX/relpos2D', Pose2D,  queue_size=10)

        self.hpposllh = Vector3()
        self.relpos2D = Pose2D()

    def run(self):

        parser = Parser([ NAV_CLS, ACK_CLS ])
        rate = rospy.Rate(10) # 10 Hz

        while(not rospy.is_shutdown()):
            try:
                msg = parser.receive_from(self.port)
                if msg[1] == "RELPOSNED":
                    self.relpos2D.x     = float(msg[2][3])/100
                    self.relpos2D.y     = float(msg[2][4])/100
                    self.relpos2D.theta = float(msg[2][7])*1e-5

                    self.relpos2D_pub.publish( self.relpos2D )

                elif msg[1] == "HPPOSLLH":
                    self.hpposllh.x     = float(msg[2][2])*1e-7
                    self.hpposllh.y     = float(msg[2][3])*1e-7
                    self.hpposllh.z     = float(msg[2][4])*1e-3

                    self.hpposllh_pub.publish( self.hpposllh )

            except (ValueError, IOError) as err:
                print(err)

            rate.sleep()

        self.port.close()


if __name__ == '__main__':
    UBX = UBXNode()
    UBX.run()
