import os
import can
import time
import numpy
import datetime
import pickle

sync_id = 0x080
sync_gen_id = 0x601
tpdo_id1 = 0x181
tpdo_id2 = 0x182
tpdo_id3 = 0x183
tpdo_id4 = 0x184
now = datetime.datetime.now()
interval = 0.0001
data_save = []

print('Bring up CAN0....')
os.system("sudo /sbin/ip link set can0 up type can bitrate 1000000")
time.sleep(0.1)

try:
    bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
except OSError:
    print('Cannot find PiCAN board.')
    exit()

print('Press CTL-C to exit')

# sync time
'''
msg = can.Message(arbitration_id=sync_gen_id, data=[0x23, 0x06, 0x10, 0x00, 0xe8, 0x03, 0x00, 0x00], extended_id=False)
bus.send(msg)
'''
msg = can.Message(arbitration_id=sync_gen_id, data=[0x23, 0x05, 0x10, 0x00, 0x80, 0x00, 0x00, 0x40], extended_id=False)
bus.send(msg)
message0 = bus.recv()
timestamp0 = message0.timestamp
t1, t2, t3, t4 = timestamp0, timestamp0, timestamp0, timestamp0
try:
    while True:
        # t0 = time.time()
        d = bus.recv()
        data_save.append({'timestamp': d.timestamp,
                          'arbitration_id': d.arbitration_id,
                          'data': d.data
                          })
        # t1 = time.time()
        # print('{0}'.format((t1-t0)*1000))
        continue

except KeyboardInterrupt:
    msg = can.Message(arbitration_id=sync_gen_id, data=[0x23, 0x05, 0x10, 0x00, 0x80, 0x00, 0x00, 0x00], extended_id=False)
    bus.send(msg)
    os.system('sudo /sbin/ip link set can0 down')
    f = open('/home/necst/can/data/tiltmeter-{0:%Y%m%d-%H%M%S}.pickle'.format(now), 'wb')
    pickle.dump(data_save, f)
    f.close()
    print('\n\rKeyboard interrtupt')
