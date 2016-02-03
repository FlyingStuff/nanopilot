import time
import sys
import zmqmsgbus

bus = zmqmsgbus.Bus(sub_addr='ipc://ipc/source',
                    pub_addr='ipc://ipc/sink')

node = zmqmsgbus.Node(bus)

i = 0
while 1:
    try:
        print('ping {}'.format(i))
        i += 1
        print(i, node.call('/{}/ping'.format(sys.argv[1]), i))
    except zmqmsgbus.call.CallFailed as e:
        print(e)
    time.sleep(0.1)
