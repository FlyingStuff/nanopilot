import rclpy
from autopilot_msgs.srv import SendMsgpackConfig
import msgpack
import yaml
import sys
from pprint import pprint
import time
import argparse


def dict_diff(d1, d2):
    diff = {}
    for key2, val2 in d2.items():
        if key2 not in d1:
            diff[key2] = val2
        else:
            if isinstance(val2, dict):
                sub_diff = dict_diff(d1[key2], val2)
                if sub_diff:
                    diff[key2] = sub_diff
            elif val2 != d1[key2]:
                diff[key2] = val2
    return diff


def send_config(node, cli, cfg_dict):
    pprint(cfg_dict)
    msg = msgpack.packb(cfg_dict, use_single_float=True)
    req = SendMsgpackConfig.Request()
    req.msgpack_config = [bytes((b, )) for b in msg]

    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        ret = future.result()
        if ret.success:
            node.get_logger().info('config sent')
        else:
            node.get_logger().error('error: ' + ret.error_message)
    else:
        node.get_logger().error('Service call failed %r' % (future.exception(),))


def split_dict_items_and_send(node, cli, cfg_dict):
    for key, val in cfg_dict.items():
        if isinstance(val, dict):
            prefix_val = {key + '/' + k: v for k, v in val.items()}
            split_dict_items_and_send(node, cli, prefix_val)
        else:
            send_config(node, cli, {key: val})
            time.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('config_client')
    cli = node.create_client(SendMsgpackConfig, 'send_config')

    parser = argparse.ArgumentParser(description='Low level controller parameter loading')
    parser.add_argument('filename', help='yaml parameter file')
    parser.add_argument('--init', help='send all parameters', action='store_true')

    args = parser.parse_args()

    filename = args.filename
    prev_parameters = {}
    if (args.init):
        print('sending all parameters')
        prev_parameters = {}
    else:
        prev_parameters = yaml.load(open(filename))
    while True:
        parameters = yaml.load(open(filename))
        parameters_diff = dict_diff(prev_parameters, parameters)

        if not parameters_diff:
            time.sleep(0.1)
            continue

        # pprint(parameters_diff)
        prev_parameters = parameters

        split_dict_items_and_send(node, cli, parameters_diff)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
