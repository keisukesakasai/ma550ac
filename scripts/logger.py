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


class logger(object):

    def __init__(self):
        self.fn = ''
        self.xtilt = [0.] * 4
        self.ytilt = [0.] * 4
        self.ztilt = [0.] * 4
        self.temp = [0.] * 4

    def callback_xtilt(self, req, idx):

        self.xtit[idx] = req.data
        return

    def callback_ytilt(self, req, idx):

        self.ytit[idx] = req.data
        return

    def callback_ztilt(self, req, idx):

        self.ztit[idx] = req.data
        return

    def callback_temp(self, req, idx):

        self.temp[idx] = req.data
        return

    def log(self):
        

if __name__ == '__main__':
    xtilt_sub_list = [rospy.Subscriber('/ma550ac_nid{0}_xtilt_arcsec'.format(nid),
                                       Float64,
                                       st.callback_xtilt,
                                       callback_args = idx)
                      for idx, nid in enumerate(range(4), start=1)]
    ytilt_sub_list = [rospy.Subscriber('/ma550ac_nid{0}_ytilt_arcsec'.format(nid),
                                       Float64,
                                       st.callback_ytilt,
                                       callback_args = idx)
                      for idx, nid in enumerate(range(4), start=1)]
    ztilt_sub_list = [rospy.Subscriber('/ma550ac_nid{0}_ztilt_arcsec'.format(nid),
                                       Float64,
                                       st.callback_ztilt,
                                       callback_args = idx)
                      for idx, nid in enumerate(range(4), start=1)]
    temp_sub_list = [rospy.Subscriber('/ma550ac_nid{0}_temp_degC'.format(nid),
                                       Float64,
                                       st.callback_temp,
                                       callback_args = idx)
                      for idx, nid in enumerate(range(4), start=1)]
