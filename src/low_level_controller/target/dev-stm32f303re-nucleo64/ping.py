import serial
import serial_datagram
import sys
import time
import statistics

dev = serial.Serial(sys.argv[1], 115200)

# dev.write(serial_datagram.encode(b'\x42\xaa\x00'+b'hello'))
# dev.write(serial_datagram.encode(b'\x42\xaa\x00'+b'hello'))
dev.write(serial_datagram.encode(b'\x42\xaa\x00'+b'hello'))
# print("write done")

s = []
for i in range(1000):
    t1 = time.time()
    dev.write(serial_datagram.encode(b'\x42\xaa\x00'+b'hello'*10))
    res = serial_datagram.read(dev)
    t2 = time.time()
    s.append(t2-t1)
    # print(res)
print('mean {} ms'.format(statistics.mean(s)*1000))
print('std {} ms'.format(statistics.stdev(s)*1000))
print('max {} ms'.format(max(s)*1000))
print('below 1 ms: {}'.format(sum(1 for i in s if i < 0.001)/len(s)))
print('below 2 ms: {}'.format(sum(1 for i in s if i < 0.002)/len(s)))

