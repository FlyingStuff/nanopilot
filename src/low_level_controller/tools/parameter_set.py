import time
import zmqmsgbus
import argparse
import yaml
import os


def parameter_send(zmqmsgbus_node, dest_node, parameter_file):
    parameters = yaml.load(open(parameter_file))
    while True:
        try:
            ret = zmqmsgbus_node.call('/{}/parameter_set'.format(dest_node), parameters)
            if ret[0] != 0:
                print('ERROR:')
                print(ret[1])
            else:
                print('sent')
                print(parameters)
            break
        except zmqmsgbus.call.CallFailed as e:
            # print(e)
            time.sleep(0.1)


def main():
    parser = argparse.ArgumentParser("send parameters over zmqmsgbus")
    parser.add_argument("file", help="YAML parameter file")
    parser.add_argument("node", help="name of the node to send parameter to")
    parser.add_argument('--from', dest='from_addr',
                        default='ipc://ipc/source')
    parser.add_argument('--to', dest='to_addr',
                        default='ipc://ipc/sink')
    # parser.add_argument("--dev", help="serial port device for datagram-messages connection")
    # parser.add_argument("--baud", help="serial port baud rate", default=115200)
    parser.add_argument("-w", "--watch",
                        help="Watch parameter file for changes.",
                        action="store_true")
    args = parser.parse_args()

    # if args.dev:
    #     fdesc = serial.Serial(args.dev, args.baud)
    # else:
    #     fdesc = os.fdopen(1, 'wb')

    bus = zmqmsgbus.Bus(sub_addr=args.from_addr,
                        pub_addr=args.to_addr)
    node = zmqmsgbus.Node(bus)

    parameter_send(node, args.node, args.file)

    if args.watch:
        # print("> watching for file changes...")
        old_mtime = os.path.getmtime(args.file)
        while True:
            try:
                try:
                    mtime = os.path.getmtime(args.file)
                # Some editors delete the file before writing to it
                except FileNotFoundError:
                    pass

                if mtime != old_mtime:
                    old_mtime = mtime
                    parameter_send(node, args.node, args.file)

                time.sleep(0.1)
            except KeyboardInterrupt:
                break


if __name__ == '__main__':
    main()
