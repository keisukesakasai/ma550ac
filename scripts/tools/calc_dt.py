#!/usr/bin/env python3


import os
import sys
import time
import numpy
import datetime
import threading

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String


class calc(object):

    def __init__(self):
        self.t = [0., 0.]
        self.dt = 0.

    def callback(req):
        self.t[0] = self.t[1]
        self.t[1] = time.time()
        self.dt = round(self.t[1]-self.t[0], 2)*1000 # msec.
        print('dt={0}ms, nid1_xtilt={1}arcsec')


if __name__ == '__main__':
    sub = rospy.Subscriber('/ma550ac_nid1_xtilt_arcsec',
                           Float64,
                           callback)
    rospy.spin()
