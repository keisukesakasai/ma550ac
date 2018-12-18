#! /usr/bin/env python3


import os
import can
import time
import numpy
import datetime
import pickle

import rospy
import std_msgs

def str2list(param):
    return param.strip('[').strip(']').split(',')

class tilt_controller(object):

    def __init__(self):
        # set params.
        all_nid = 0x000
        base_nid = 0x600
        cob_mask = 0b11110000000
        cob_tpdo1 = 0b00110000000
        cob_tpdo4 = 0b10010000000
        restart_data = [0x01] + [0x00] * 7
        pre_mode_data = [0x80] + [0x00] * 7
        sync_producer_data = [0x23, 0x05, 0x10, 0x00, 0x80, 0x00, 0x00, 0x40]

        self.nid_list = str2list(rospy.get_param('~nid_list'))
        self.datafmt = rospy.get_param('~datafmt')
        self.synctime = rospy.get_param('~synctime')
        self.sync_producer_nid = rospy.get_param('~sync_producer_nid')

        # CAN interface up.
        pirnt('Bring up CAN0 interface...')
        os.system("sudo /sbin/ip link set can0 up type can bitrate 1000000")
        time.sleep(1e-1)
        try:
            self.bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
        except OSError:
            print('Cannot find PiCAN board.')
            exit()

        # define ROS parameter.
        arbitration_id_list = [cob_tpdo1 + nid for nid in self.nid_list] + [cob_tpdo4 + nid for nid in self.nid_list]
        topic_list = ['/tiltmeter_nid{}_angle'.format(nid) for nid in self.nid_list] + ['/tiltmeter_nid{}_temp'.format(nid) for nid in self.nid_list]
        pub_list = [
            {arbitration_id: rospy.Publisher(
            name = topic,
            data_class = std_msgs.msg.ByteMultiArray,
            latch = True
            queue_size = 1
            )} for arbitration_id, topic in zip(arbitration_id, topic)]

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
        msg = can.Message(arbitration_id=all_nid, data=pre_mode_data, extended_id=False)
        bus.send(msg)
        time.sleep(1) # for the time being.

        # set sync time.
        msg = can.Message(arbitration_id=self.sync_producer_nid, data=synctime_data, extended_id=False)
        bus.send(msg)
        time.sleep(1) # for the time being.

        # restart.
        msg = can.Message(arbitration_id=all_nid, data=restart_data, extended_id=False)
        self.bus.send(msg)
        time.sleep(1)

        print('[INFO] SET SYNC TIME {} msec.'.format(sync_time / 1000))
        return

    def set_datafmt(self):
        if self.datafmt == 'acc':
            fmt_data = [0x2f, 0x05, 0x20, 0x00, 0x11, 0x00, 0x00, 0x00]
        elif self.datafmt == 'tilt':
            fmt_data = [0x2f, 0x05, 0x20, 0x00, 0x21, 0x00, 0x00, 0x00]
        else: pass

        # pre-perational mode.
        msg = can.Message(arbitration_id=all_nid, data=pre_mode_data, extended_id=False)
        bus.send(msg)
        time.sleep(1) # for the time being.

        # set data format.
        for _nid in range(1, len(self.nid_list)+1):
            msg = can.Message(arbitration_id=base_nid+_nid, data=fmt_data, extended_id=False)
            self.bus.send(msg)
            time.sleep(1) # for the time being.

        # restart.
        msg = can.Message(arbitration_id=anode, data=data_restart, extended_id=False)
        self.bus.send(msg)
        time.sleep(1) # for the time being.

        print('[INFO] SET DATA FORMAT : {}'.format(self.datafmt))
        return

    def start_syncmode(self):
        # start syncmode
        msg = can.Message(arbitration_id=sync_producer_nid, data=sync_producer_data, extended_id=False)
        self.bus.send(msg)

        #

if __name__ == '__main__':
    rospy.init_node('tiltmeter')
    # self.set_synctime()
    # self.set_datafmt()
    ctrl = tilt_controller()
    # ctrl.start_thread_ROS()
    # rospy.spin()
