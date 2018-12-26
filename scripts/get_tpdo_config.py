import os
import sys
import can
import time
import pprint

cnt_node = int(sys.argv[1])
all_nid = 0x000
base_nid = 0x600
restart_data = [0x81] + [0x00] * 7

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
    arbitration_id=all_nid,
    data=restart_data,
    extended_id=False
    )
bus.send(msg) # restart
for i in range(cnt_node+1):
    d = bus.recv()
    print(d)
    if d.arbitration_id == 0x000: continue
    nid = int(hex(d.arbitration_id)[-1])
    nid_list.append(nid)

print(sorted(nid_list))
