import os
import sys
import can
import time

anode = 0x000
nid = 0x601
sync_time = int(float(sys.argv[1]) * 1e3) # usec.

def time2data(t):
    _hex = hex(t)
    _hex = _hex.replace('0x', '0'*(10-len(_hex)))
    lhex = [(i+j) for (i,j) in zip(_hex[::2], _hex[1::2])]
    lhex.reverse()

    return lhex

data_sync_time = [0x23, 0x06, 0x10, 0x00]
data_sync_time.extend([int(_, 16) for _ in time2data(sync_time)])
data_pre_mode = [0x80] + [0x00] * 7
data_restart = [0x01] + [0x00] * 7

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

# set sync time
msg = can.Message(arbitration_id=nid, data=data_sync_time, extended_id=False)
bus.send(msg)
time.sleep(1)

# restart
msg = can.Message(arbitration_id=anode, data=data_restart, extended_id=False)
bus.send(msg)
time.sleep(1)

os.system('sudo /sbin/ip link set can0 down')
print('[INFO] SET SYNC TIME {} msec.'.format(sync_time / 1000))
        
