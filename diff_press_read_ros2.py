from smbus2 import SMBusWrapper, i2c_msg
import time
import sys
import struct
import threading
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import FluidPressure


class SDP3xDriver(Node):
    def __init__(self, i2c_bus, i2c_addr, name='SDP3x_driver', topic='pressure'):
        super().__init__(name)
        self.bus = i2c_bus
        self.addr = i2c_addr
        self.publisher = self.create_publisher(FluidPressure, topic, 10)
        timer_period = 1/50  # seconds
        self.timer = self.create_timer(timer_period, self.read_and_publish)
        self.is_initialized = False

    def setup(self):
        # stop continuous mode
        msg = i2c_msg.write(self.addr, [0x3F, 0xF9])
        self.bus.i2c_rdwr(msg)

        time.sleep(0.1)

        # setup continuous mode
        msg = i2c_msg.write(self.addr, [0x36, 0x15])
        self.bus.i2c_rdwr(msg)

        time.sleep(0.1)

        msg = i2c_msg.read(self.addr, 9)
        self.bus.i2c_rdwr(msg)
        msgb = bytes(list(msg))
        self.press_res = struct.unpack_from('>h', msgb, 6)[0]

        time.sleep(0.01)
        self.is_initialized = True

    def read_and_publish(self):
        try:
            if self.is_initialized:
                msg = i2c_msg.read(self.addr, 6)
                self.bus.i2c_rdwr(msg)
                msgb = bytes(list(msg))
                press = struct.unpack_from('>h', msgb, 0)[0]/self.press_res
                temp = struct.unpack_from('>h', msgb, 3)[0]/200
                msg = FluidPressure()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.fluid_pressure = press
                self.publisher.publish(msg)
            else:
                self.setup()
        except OSError as err:
            print(err)
            self.is_initialized = False


class LockedSMBusWrapper:
    def __init__(self, bus):
        self.bus = bus
        self.lock = threading.Lock()

    def i2c_rdwr(self, msg):
        with self.lock:
            self.bus.i2c_rdwr(msg)


def main(args=None):
    rclpy.init(args=args)
    bus_nbr = 0
    with SMBusWrapper(bus_nbr) as bus:
        locked_bus = LockedSMBusWrapper(bus)
        executor = MultiThreadedExecutor(num_threads=4)
        nodes = []
        nodes.append(SDP3xDriver(locked_bus, 0x21, 'sdp3x_dynamic_pressure', '/airdata/dynamic_pressure'))
        nodes.append(SDP3xDriver(locked_bus, 0x22, 'sdp3x_aoa_pressure', '/airdata/aoa_pressure'))
        nodes.append(SDP3xDriver(locked_bus, 0x23, 'sdp3x_aos_pressure', '/airdata/aos_pressure'))
        for n in nodes:
            executor.add_node(n)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            for n in nodes:
                n.destroy_node()

if __name__ == '__main__':
    main()
