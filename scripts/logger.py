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


# --
name = 'tilt_logger'
data_dir = '/home/amigos/data/experiments/'
save_dir = os.path.join(data_dir, name)
# --

class logger(object):

    def __init__(self):
        self.average = 1
        self.synctime = 2e-3 # 2 msec.
        self.xtilt = [0.] * 4
        self.ytilt = [0.] * 4
        self.ztilt = [0.] * 4
        self.temp = [0.] * 4
        self.datetime_buff = []
        self.xtilt_buff = []
        self.ytilt_buff = []
        self.ztilt_buff = []
        self.temp_buff = []
        exp_time = datetime.datetime.fromtimestamp(float(time.time()))
        ymd = exp_time.strftime('%Y%m%d_')
        hms = exp_time.strftime('%H%M%S')
        self.saveto = os.path.join(save_dir, ymd + hms)
        os.makedirs(self.saveto)
        self.filename_datetime = self.saveto + '/datetime.txt'
        self.filename_xtilt = self.saveto + '/xtilt.txt'
        self.filename_ytilt = self.saveto + '/ytilt.txt'
        self.filename_ztilt = self.saveto + '/ztilt.txt'
        self.filename_temp = self.saveto + '/temp.txt'
        print('[INFO] File open (dir: {})'.format(self.saveto))

        f_datetime = open(self.filename_datetime, 'a')
        f_xtilt = open(self.filename_xtilt, 'a')
        f_ytilt = open(self.filename_ytilt, 'a')
        f_ztilt = open(self.filename_ztilt, 'a')
        f_temp = open(self.filename_temp, 'a')

        f_datetime.close()
        f_xtilt.close()
        f_ytilt.close()
        f_ztilt.close()
        f_temp.close()

    def callback_xtilt(self, req, idx):

        self.xtilt[idx] = req.data
        return

    def callback_ytilt(self, req, idx):

        self.ytilt[idx] = req.data
        return

    def callback_ztilt(self, req, idx):

        self.ztilt[idx] = req.data
        return

    def callback_temp(self, req, idx):

        self.temp[idx] = req.data
        return

    def log(self):
        while not rospy.is_shutdown():
            for i in range(self.average):
                self.datetime_buff.append(time.time())
                self.xtilt_buff.append(self.xtilt)
                self.ytilt_buff.append(self.ytilt)
                self.ztilt_buff.append(self.ztilt)
                self.temp_buff.append(self.temp)
                time.sleep(self.synctime) # 2 msec.

            datetime_ave = numpy.mean(self.datetime_buff)
            xtilt_ave = [numpy.array(self.xtilt_buff)[:,i].mean() for i in range(4)]
            ytilt_ave = [numpy.array(self.ytilt_buff)[:,i].mean() for i in range(4)]
            ztilt_ave = [numpy.array(self.ztilt_buff)[:,i].mean() for i in range(4)]
            temp_ave = [numpy.array(self.temp_buff)[:,i].mean() for i in range(4)]
            self.xtilt_buff = []
            self.ytilt_buff = []
            self.ztilt_buff = []
            self.temp_buff = []

            datetime = str(time.time()) + '\n'
            xtilt = ' '.join(map(str, xtilt_ave)) + '\n'
            ytilt = ' '.join(map(str, ytilt_ave)) + '\n'
            ztilt = ' '.join(map(str, ztilt_ave)) + '\n'
            temp = ' '.join(map(str, temp_ave)) + '\n'

            f_datetime = open(self.filename_datetime, 'a')
            f_xtilt = open(self.filename_xtilt, 'a')
            f_ytilt = open(self.filename_ytilt, 'a')
            f_ztilt = open(self.filename_ztilt, 'a')
            f_temp = open(self.filename_temp, 'a')

            f_datetime.write(datetime)
            f_xtilt.write(xtilt)
            f_ytilt.write(ytilt)
            f_ztilt.write(ztilt)
            f_temp.write(temp)

            f_datetime.close()
            f_xtilt.close()
            f_ytilt.close()
            f_ztilt.close()
            f_temp.close()

            time.sleep(1e-3) # 1 msec.

    def start_thread(self):
        th = threading.Thread(target=self.log)
        th.setDaemon(True)
        th.start()


if __name__ == '__main__':
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
        pass

    st = logger()
    st.start_thread()
    rospy.init_node(name)
    xtilt_sub_list = [rospy.Subscriber('/ma550ac_nid{0}_xtilt_arcsec'.format(nid),
                                       Float64,
                                       st.callback_xtilt,
                                       callback_args = idx)
                      for nid, idx in enumerate(range(4), start=1)]
    ytilt_sub_list = [rospy.Subscriber('/ma550ac_nid{0}_ytilt_arcsec'.format(nid),
                                       Float64,
                                       st.callback_ytilt,
                                       callback_args = idx)
                      for nid, idx in enumerate(range(4), start=1)]
    ztilt_sub_list = [rospy.Subscriber('/ma550ac_nid{0}_ztilt_arcsec'.format(nid),
                                       Float64,
                                       st.callback_ztilt,
                                       callback_args = idx)
                      for nid, idx in enumerate(range(4), start=1)]
    temp_sub_list = [rospy.Subscriber('/ma550ac_nid{0}_temp_degC'.format(nid),
                                       Float64,
                                       st.callback_temp,
                                       callback_args = idx)
                      for nid, idx in enumerate(range(4), start=1)]
    rospy.spin()

