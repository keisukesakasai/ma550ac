import os
import sys
import can
import time
import pprint

cnt_node = int(sys.argv[1])
all_nid = 0x000
base_nid = 0x600
restart_data = [0x81] + [0x00] * 7
read_tpdo_data = [[0x40] + 0x00 + i + [0x18, 0x01, 0x00, 0x00, 0x00, 0x00]
                  for i in range(4)]

nid_list = []
tpdo_list = []


print('Bring up CAN0....')
os.system("sudo /sbin/ip link set can0 up type can bitrate 1000000")
time.sleep(1e-1)

try:
    bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
except OSError:
    print('Cannot find PiCAN board.')
    exit()

# get node-ID.
msg = can.Message(
    arbitration_id=all_nid,
    data=restart_data,
    extended_id=False
    )
bus.send(msg) # restart
for i in range(cnt_node):
    d = bus.recv()
    if d.arbitration_id == 0x000: continue
    nid = int(hex(d.arbitration_id)[-1])
    nid_list.append(nid)
nid_list = sorted(nid_list)

# get topo config.
for nid in nid_list:
    _tpdo_list = []
    for data in read_tpdo_data:
        msg = can.Message(
            arbitration_id=base_nid+nid,
            data=data,
            extended_id=False)
        bus.send(msg)
        d = bus.recv().data
        _tpdo_list.append(d[-1])
    tpdo_list.append(_tpdo_list)

