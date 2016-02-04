import time
import zmqmsgbus
import argparse
import yaml


def main():
    parser = argparse.ArgumentParser("send parameters over zmqmsgbus")
    parser.add_argument("service", help="service to call")
    parser.add_argument("arg", help="YAML call arguments")
    parser.add_argument('--in', dest='from_addr',
                        default='ipc://ipc/source')
    parser.add_argument('--out', dest='to_addr',
                        default='ipc://ipc/sink')
    args = parser.parse_args()

    bus = zmqmsgbus.Bus(sub_addr=args.from_addr,
                        pub_addr=args.to_addr)
    node = zmqmsgbus.Node(bus)

    while 1:
        try:
            a = yaml.load(args.arg)
            print('calling {} with {}'.format(args.service, a))
            ret = node.call(args.service, a)
            print(ret)
            break
        except zmqmsgbus.call.CallFailed as e:
            print(e)
            time.sleep(0.1)

if __name__ == '__main__':
    main()
