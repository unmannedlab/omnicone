#!/usr/bin/env python
import serial
import pynmea2
import rospy
import math
from geometry_msgs.msg import Vector3


def ddmmstr_2_degflt(ddmm_str):
    # Split input string. Assumes input is of form DDDDMM.MMMM
    # Returns float(DDDD) + float(MM.MMMM / 60)
    deg_float = float(ddmm_str[0:ddmm_str.find('.')-2])
    min_float = float(ddmm_str[  ddmm_str.find('.')-2:len(ddmm_str)])
    return (deg_float + min_float / 60)


class  gps_node:
    # Class for accessing and publishing data from Ardusimple RTK
    def __init__(self):
        # Initializes node and publisher. Loops through each possible port to
        #     find the device that outputs expected GPS data. This fixes issue
        #     with /dev/ttyACM# ports being undefined after a power cycle.

        rospy.init_node('gps_publisher')
        self.pub = rospy.Publisher('gps_pub', Vector3, queue_size = 10 )

        exit_flag = False
        while not exit_flag:
            for port in ["/dev/ttyACM0","/dev/ttyACM1","/dev/ttyACM2"]:
                try:
                    self.serial_port = serial.Serial(port, baudrate=115200, timeout=3)
                    self.NMEA_str = self.serial_port.readline()

                    if self.NMEA_str.find('GNRMC') > 0:
                        exit_flag = True
                        break

                except serial.SerialException:
                    pass


    def ParseGPS(self):
        try:
            if self.NMEA_str.find('GGA') > 0:

                msg = pynmea2.parse(self.NMEA_str)

                lat_flt = ddmmstr_2_degflt(msg.lat)
                lon_flt = ddmmstr_2_degflt(msg.lon)

                alt_flt = float(msg.altitude)

                if msg.lat_dir == 'S':
                    lat_flt *= -1.0
                if msg.lon_dir == 'W':
                    lon_flt *= -1.0

                self.msg = Vector3(lat_flt, lon_flt, alt_flt)

            else:

                self.msg = None

        except pynmea2.nmea.ParseError:
            print('Failed to parse line, consider lowering baud rate')
            self.msg = None

        except ValueError:
            print('Value Error, returning none')
            self.msg = None


    def run(self):
        rate = rospy.Rate(10) # 10 Hz

        while(not rospy.is_shutdown()):
            try:
                self.NMEA_str = self.serial_port.readline()
                if self.NMEA_str != None:
                    print self.NMEA_str
                    self.ParseGPS()

                    if self.msg != None:
                        self.pub.publish(self.msg)

            except KeyboardInterrupt:
                print('\nKeyboard Interrupt')
                return 0


if __name__ == '__main__':
    gps_node_pub = gps_node()
    gps_node_pub.run()
