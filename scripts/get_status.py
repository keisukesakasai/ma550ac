import os
import can
import time
import pprint
import binascii

base_nid = 0x600
master_nid = 0x601

data_fmt = {'Accelerometer': 10,
            'Tile_angle_sensor': 20}

data_read_synctime = [0x40, 0x06, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00]
data_read_filefmt = [0x40, 0x05, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00]

print('Bring up CAN0....')
os.system("sudo /sbin/ip link set can0 up type can bitrate 1000000")
time.sleep(1e-1)

try:
    bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
except OSError:
    print('Cannot find PiCAN board.')
    exit()

d = {}

read_synctime = can.Message(arbitration_id=master_nid, data=data_read_synctime, extended_id=False)
bus.send(read_synctime)
_synctime = binascii.hexlify(bus.recv().data)
synctime = int(_synctime[14:16] + _synctime[12:14] + _synctime[10:12] + _synctime[8:10], 16)
d['synctime (msec.)'] = float(synctime / 1000)

# for i in range(1, 4 + 1):
for i in [1, 3, 4]:
      read_filefmt = can.Message(arbitration_id=base_nid+i, data=data_read_filefmt, extended_id=False)
      bus.send(read_filefmt)
      filefmt = int(binascii.hexlify(bus.recv().data)[8:10])
      idx = list(data_fmt.values()).index(filefmt)
      d['filefmt_nid{}'.format(i)] = list(data_fmt.keys())[idx]

print('\n/-------------data_format-------------/\n')
pprint.pprint(d)
print('\n/-------------------------------------/')
