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
        self.nid = rospy.get_param('~nid')
        self.datafmt = rospy.get_param('datafmt')
        self.synctime = rospy.get_param('~synctime')
        self.sync_producer_nid = rospy.get_param('~sync_producer_nid')
        print(self.nid)
        print(self.datetime)
        print(self.synctime)
        print(self.sync_producer_nid)

if __name__ == '__main__':
    rospy.init_node('tiltmeter')
    ctrl = tilt_controller()
    ctrl.start_thread_ROS()
    rospy.spin()
