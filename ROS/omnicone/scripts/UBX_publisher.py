#!/usr/bin/env python3
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

        self.hpposllh_llh_pub  = rospy.Publisher('UBX/hpposllh/llh', Vector3, queue_size=10)
        self.hpposllh_err_pub  = rospy.Publisher('UBX/hpposllh/err', Vector3, queue_size=10)
        self.relpos2D_pos_pub  = rospy.Publisher('UBX/relpos2D/pos', Pose2D,  queue_size=10)
        self.relpos2D_err_pub  = rospy.Publisher('UBX/relpos2D/err', Pose2D,  queue_size=10)

        self.hpposllh_llh = Vector3()
        self.hpposllh_err = Vector3()
        self.relpos2D_pos = Pose2D()
        self.relpos2D_err = Pose2D()

    def run(self):

        parser = Parser([ NAV_CLS, ACK_CLS ])
        rate = rospy.Rate(10) # 10 Hz

        while(not rospy.is_shutdown()):
            try:
                msg = parser.receive_from(self.port)
                if msg[1] == "RELPOSNED":
                    self.relpos2D_pos.x     = float(msg[2][3])/100
                    self.relpos2D_pos.y     = float(msg[2][4])/100
                    self.relpos2D_pos.theta = float(msg[2][7])*1e-5

                    self.relpos2D_err.x     = float(msg[2][12])*1e-4
                    self.relpos2D_err.y     = float(msg[2][13])*1e-4
                    self.relpos2D_err.theta = float(msg[2][16])*1e-5

                    self.relpos2D_pos_pub.publish( self.relpos2D_pos )
                    self.relpos2D_err_pub.publish( self.relpos2D_err )

                elif msg[1] == "HPPOSLLH":
                    self.hpposllh_llh.x     = float(msg[2][2])*1e-7
                    self.hpposllh_llh.y     = float(msg[2][3])*1e-7
                    self.hpposllh_llh.z     = float(msg[2][4])*1e-3

                    self.hpposllh_err.x     = float(msg[2][10])*1e-7
                    self.hpposllh_err.y     = float(msg[2][10])*1e-7
                    self.hpposllh_err.z     = float(msg[2][11])*1e-4

                    self.hpposllh_llh_pub.publish( self.hpposllh_llh )
                    self.hpposllh_err_pub.publish( self.hpposllh_err )

            except (ValueError, IOError) as err:
                print(err)

            rate.sleep()

        self.port.close()


if __name__ == '__main__':
    UBX = UBXNode()
    UBX.run()
