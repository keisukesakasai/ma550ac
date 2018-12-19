#! /usr/bin/env python3


import os
import sys
sys.path.append('/home/necst/.pyenv/versions/3.6.4/lib/python3.6/site-packages')
import can
import time
import numpy
import datetime
import pickle
import threading
import signal

import rospy
import std_msgs


def str2list(param):
    return param.strip('[').strip(']').split(',')


class ma550ac_controller(object):

    def __init__(self):
        # set params.
        self.all_nid = 0x000
        self.base_nid = 0x600
        cob_mask = 0b11110000000
        cob_tpdo1 = 0b00110000000
        cob_tpdo4 = 0b10010000000
        self.restart_data = [0x01] + [0x00] * 7
        self.pre_mode_data = [0x80] + [0x00] * 7
        self.sync_producer_start_data = [0x23, 0x05, 0x10, 0x00, 0x80, 0x00, 0x00, 0x40]
        self.sync_producer_stop_data = [0x23, 0x05, 0x10, 0x00, 0x80, 0x00, 0x00, 0x00]

        self.nid_list = list(map(int, str2list(rospy.get_param('~nid_list'))))
        self.datafmt = rospy.get_param('~datafmt')
        self.synctime = rospy.get_param('~synctime')
        self.sync_producer_nid = self.base_nid + rospy.get_param('~sync_producer_nid')

        # define ROS parameter.
        self.arbitration_id_list = [cob_tpdo1 + nid for nid in self.nid_list] + [cob_tpdo4 + nid for nid in self.nid_list]
        topic_list = ['/ma550ac_nid{0}_{1}_binary'.format(nid, self.datafmt) for nid in self.nid_list] + ['/ma550ac_nid{}_temp_binary'.format(nid) for nid in self.nid_list]
        self.pub_list = [rospy.Publisher(
            name = topic,
            data_class = std_msgs.msg.Int64MultiArray,
            latch = True,
            queue_size = 1
            ) for topic in topic_list]

        # CAN interface up.
        print('Bring up CAN0 interface...')
        os.system("sudo /sbin/ip link set can0 up type can bitrate 1000000")
        time.sleep(1e-1)
        try:
            self.bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
        except OSError:
            print('Cannot find PiCAN board.')
            exit()

        # define event handler.
        signal.signal(signal.SIGINT, self.handler)

    def handler(self, signal, frame):
        msg = can.Message(
            arbitration_id=self.sync_producer_nid,
            data=self.sync_producer_stop_data,
            extended_id=False
            )
        self.bus.send(msg)
        os.system('sudo /sbin/ip link set can0 down')
        print('\n\rKeyboard interrtupt')

    def set_synctime(self):
        synctime = self.synctime * 1000 # usec.

        def time2data(t):
            """
            sync time [usec.] --> can data format [8 byte]
            """
            _hex = hex(t)
            _hex = _hex.replace('0x', '0'*(10-len(_hex)))
            lhex = [(i+j) for (i,j) in zip(_hex[::2], _hex[1::2])]
            lhex.reverse()

            return lhex

        synctime_data = [0x23, 0x06, 0x10, 0x00]
        synctime_data.extend([int(_, 16) for _ in time2data(synctime)])

        # pre-operational mode.
        msg = can.Message(
            arbitration_id=self.all_nid,
            data=self.pre_mode_data,
            extended_id=False
            )
        self.bus.send(msg)
        time.sleep(1) # for the time being.

        # set sync time.
        msg = can.Message(
            arbitration_id=self.sync_producer_nid,
            data=synctime_data,
            extended_id=False
            )
        self.bus.send(msg)
        time.sleep(1) # for the time being.

        # restart.
        msg = can.Message(
            arbitration_id=self.all_nid,
            data=self.restart_data,
            extended_id=False
            )
        self.bus.send(msg)
        time.sleep(1)

        print('[INFO] Set sync time {} msec.'.format(synctime / 1000))
        return

    def set_datafmt(self):
        if self.datafmt == 'acc':
            fmt_data = [0x2f, 0x05, 0x20, 0x00, 0x11, 0x00, 0x00, 0x00]
        elif self.datafmt == 'tilt':
            fmt_data = [0x2f, 0x05, 0x20, 0x00, 0x21, 0x00, 0x00, 0x00]
        else: pass

        # pre-perational mode.
        msg = can.Message(
            arbitration_id=self.all_nid,
            data=self.pre_mode_data,
            extended_id=False
            )
        self.bus.send(msg)
        time.sleep(1) # for the time being.

        # set data format.
        for _nid in self.nid_list:
            msg = can.Message(
                arbitration_id=self.base_nid+_nid,
                data=fmt_data,
                extended_id=False
                )
            self.bus.send(msg)
            time.sleep(1) # for the time being.

        # restart.
        msg = can.Message(
            arbitration_id=self.all_nid,
            data=self.restart_data,
            extended_id=False
            )
        self.bus.send(msg)
        time.sleep(1) # for the time being.

        print('[INFO] Set data format : {}'.format(self.datafmt))
        return

    def start_syncmode(self):
        # start syncmode
        msg = can.Message(
            arbitration_id=self.sync_producer_nid,
            data=self.sync_producer_start_data,
            extended_id=False
            )
        self.bus.send(msg)

        # publish
        while not rospy.is_shutdown():
            d = self.bus.recv()
            if d.arbitration_id in self.arbitration_id_list:
                msg = std_msgs.msg.Int64MultiArray()
                msg.data = [_d for _d in d.data]
                pub_idx = self.arbitration_id_list.index(d.arbitration_id)
                self.pub_list[pub_idx].publish(msg)
                time.sleep(1e-3)
            else: pass


if __name__ == '__main__':
    rospy.init_node('ma550ac')
    ctrl = ma550ac_controller()
    ctrl.set_synctime()
    ctrl.set_datafmt()
    time.sleep(1)
    print('[INFO] Start M-A550AC data pubilsh...')
    ctrl.start_syncmode()
