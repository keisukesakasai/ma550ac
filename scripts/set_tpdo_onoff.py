import os
import sys
import can
import time


_nid = sys.argv[1]
tpdo = sys.argv[2]
onoff = sys.argv[3]

all_nid = 0x000
base_nid = 0x600
nid = base_nid + _nid
restart_data = [0x01] + [0x00] * 7
pre_mode_data = [0x80] + [0x00] * 7
save_parameter_data = [0x23, 0x10, 0x10, 0x01, 0x73, 0x61, 0x76, 0x65]

if onoff == 1:
    txt = 'valid'
    onoff_data = [0x23, 0x01, 0x18, 0x01, 0x81, 0x00+tpdo, 0x00, 0x40]
elif onoff == 0:
    txt = 'invalid'
    onoff_data = [0x23, 0x01, 0x18, 0x01, 0x81, 0x00+tpdo, 0x00, 0xc0]


print('Bring up CAN0....')
os.system("sudo /sbin/ip link set can0 up type can bitrate 1000000")
time.sleep(1e-1)

try:
    bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
except OSError:
    print('Cannot find PiCAN board.')
    exit()


# pre-operational mode.
msg = can.Message(
    arbitration_id=all_nid,
    data=pre_mode_data,
    extended_id=False)
bus.send(msg)
time.sleep(5e-2)

# set tpdo onoff.
msg = can.Message(
    arbitration_id=nid,
    data=onoff_data,
    extended_id=False)
bus.send(msg)

# save all parameters.
msg = can.Message(
    arbitration_id=nid,
    data=save_parameter_data,
    extended_id=False)
bus.send(msg)
time.sleep(5e-2)

# restart.
msg = can.Message(
    arbitration_id=all_nid,
    data=restart_data,
    extended_id=False)
bus.send(msg)
time.sleep(5e-2)

print('[INFO] Tiltmeter nid : {0}\n'
      '       TPDO{1} : {2}'.format(_nid, tpdo, txt))
