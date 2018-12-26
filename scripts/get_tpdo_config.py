import os
import can
import time
import pprint
import datetime

all_nid = 0x000
base_nid = 0x600
restart_data = [0x01] + [0x00] * 7

nid_list =[]

print('Bring up CAN0....')
os.system("sudo /sbin/ip link set can0 up type can bitrate 1000000")
time.sleep(1e-1)

try:
    bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
except OSError:
    print('Cannot find PiCAN board.')
    exit()

# get node-ID
msg = can.Message(
    arbitration_id=all_node,
    data=restart_data,
    extended_id=False
    )
bus.send(msg)
t0 = datatime.datatime.now()
t1 = t0 + 1. # 1 sec. after
while not(t0 == t1):
    d = bus.recv()
    nid = hex(d.arbitration_id)[-1]
    nid_list.append(nid)
    t0 = datatime.datatie.now()

print(sorted(nid_list))
