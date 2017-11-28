import serial
import serial_datagram
import sys

dev = serial.Serial(sys.argv[1], 115200)

dev.write(serial_datagram.encode(b'\x43\xaa\x00'+b'hello'))
while 1:
    print(serial_datagram.read(dev))

