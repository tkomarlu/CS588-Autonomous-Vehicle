#!/usr/bin/env python3

#================================================================
# File name: mp0.py
#================================================================

from __future__ import print_function

# Python Headers
import os 
import csv
import math
import time
import numpy as np
from numpy import linalg as la
import scipy.signal as signal

# ROS Headers
import alvinxy.alvinxy as axy # Import AlvinXY transformation module
import rospy

# GEM Sensor Headers
from std_msgs.msg import String, Bool, Float32, Float64
from novatel_gps_msgs.msg import NovatelPosition, NovatelXYZ, Inspva

# GEM PACMod Headers
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd, SystemRptFloat, VehicleSpeedRpt


class Blink(object):
    def __init__(self):
        # GEM vechile turn signal control
        self.turn_pub = rospy.Publisher('/pacmod/as_rx/turn_cmd', PacmodCmd, queue_size=1)
        self.turn_cmd = PacmodCmd()
        self.turn_cmd.ui16_cmd = 1 # None
        self.turn_pub.publish(self.turn_cmd)

    def flashLeft(self, n):
        self.turn_cmd.ui16_cmd = 2
        self.turn_pub.publish(self.turn_cmd)
        # self.turn_cmd.ui16_cmd = 1
        # self.turn_pub.publish(self.turn_cmd)

    def flashRight(self, n):
        self.turn_cmd.ui16_cmd = 0
        self.turn_pub.publish(self.turn_cmd)
        # self.turn_cmd.ui16_cmd = 1
        # self.turn_pub.publish(self.turn_cmd)

    def noFlash(self, n):
        self.turn_cmd.ui16_cmd = 1
        self.turn_pub.publish(self.turn_cmd)
        # self.turn_cmd.ui16_cmd = 1
        # self.turn_pub.publish(self.turn_cmd)

    def blinkSOS(self):
        while not rospy.is_shutdown():
            self.flashLeft(3)
            time.sleep(2)
            self.flashRight(3)
            time.sleep(2)
            self.flashLeft(3)
            time.sleep(2)
            self.noFlash(3)
            time.sleep(3)

def blink():

    rospy.init_node('sos_node', anonymous=True)
    pp = Blink()

    try:
        pp.blinkSOS()

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    blink()


