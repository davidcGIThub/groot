#!/usr/bin/env python
from __future__ import print_function
import rospy

from rosflight_msgs.msg import Barometer
from sensor_msgs.msg import Range

import serial
import time

class SerialReader:
    def __init__(self):
        self._baro_pub = rospy.Publisher("/serial/baro", Barometer,
                queue_size=10)
        self._laser_pub = rospy.Publisher("/serial/laser", Range,
                queue_size=10)
        while not rospy.is_shutdown():
            try:
                with serial.Serial('/dev/ttyACM0', 9600) as ser:
                    line = ser.readline()
                    idx = line.find('Pa')
                    press = float(line[0:idx])
                    #  press = (line[0:idx])
                    idx = line.find('K')
                    temp = float(line[(idx - 7):idx])
                    #  temp = (line[(idx - 7):idx])
                    idx = line.find('dist:')
                    range_str = line[idx:]
                    idx2 = range_str.find(',')
                    dist = float(range_str[6:idx2])
                    #  dist = (range_str[6:idx2])
                    #  print(line)
                    self.pub_baro(press, temp)
                    self.pub_laser(dist)
                    #  print("press: ", press)
                    #  print("temp: ", temp)
                    #  print("dist: ", dist)
            except Exception as e:
                #  print("Exception: ", e)
                rospy.loginfo_throttle(3., "Serial read failed. This might not"
                        " be an error")
                #  print("No Serial")
                #  time.sleep(0.5)

    def pub_baro(self, press, temp):
        baro_msg = Barometer()
        baro_msg.header.stamp = rospy.Time.now()
        baro_msg.pressure = press
        baro_msg.temperature = temp

        self._baro_pub.publish(baro_msg)

    def pub_laser(self, dist):
        laser_msg = Range()
        laser_msg.header.stamp = rospy.Time.now()
        laser_msg.range = dist
        laser_msg.min_range = 0.5
        laser_msg.max_range = 12.
        laser_msg.field_of_view = 0.0349
        laser_msg.radiation_type = 1

        self._laser_pub.publish(laser_msg);


if __name__ == '__main__':
    rospy.init_node('serial_read', anonymous=False)
    try:
        reader = SerialReader()
    except rospy.ROSInterruptException:
        pass

