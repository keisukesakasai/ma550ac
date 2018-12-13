import os
import can
import pprint
import binascii

base_nid = 0x060
master_nid = 0x061

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
synctime = binascii.hexlify(bus.recv().data, 'utf-8')[8:10]
d['synctime'] = synctime

for i in range(1, 4 + 1):
      read_filefmt = can.Message(arbitration_id=base_nid+i, data=data_read_filefmt, extended_id=False)
      bus.send(read_filefmt)
      _filefmt = binascii.hexlify(bus.recv().data, 'utf-8')
      filefmt = int(_filefmt[10:12] + _filefmt[8:10], hex)
      d['filefmt_nid{}'.format(i)] = filefmt

pprint.pprint(d)
