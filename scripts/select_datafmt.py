import os
import sys
import can
import time

fmt = sys.argv[1]

anode = 0x000
base_nid = 0x600
data_fmt = {'Accelerometer': 10,
            'Tile_angle_sensor': 20}

data_pre_mode = [0x80] + [0x00] * 7
data_restart = [0x01] + [0x00] * 7

if fmt == 'acc':
    data = [0x2f, 0x05, 0x20, 0x00, 0x11, 0x00, 0x00, 0x00]
elif fmt == 'tilt':
    data = [0x2f, 0x05, 0x20, 0x00, 0x21, 0x00, 0x00, 0x00]
else: pass

print('Bring up CAN0....')
os.system("sudo /sbin/ip link set can0 up type can bitrate 1000000")
time.sleep(1e-1)

try:
    bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
except OSError:
    print('Cannot find PiCAN board.')
    exit()

# pre-operational mode
msg = can.Message(arbitration_id=anode, data=data_pre_mode, extended_id=False)
bus.send(msg)
time.sleep(1)

# set data format
for _ in range(1, 4 + 1):
    msg = can.Message(arbitration_id=base_nid+_, data=data, extended_id=False)
    bus.send(msg)
    time.sleep(1)

# restart
msg = can.Message(arbitration_id=anode, data=data_restart, extended_id=False)
bus.send(msg)
time.sleep(1)

os.system('sudo /sbin/ip link set can0 down')
print('[INFO] SET DATA FORMAT : {}'.format(fmt))
