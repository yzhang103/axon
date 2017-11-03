#!/usr/bin/python
import serial
import time

Speed = 1

MESSAGE = bytearray(3)
MESSAGE[0] = 1
MESSAGE[1] = 0
MESSAGE[2] = 0

if __name__ == '__main__':
    _port = '/dev/ttyUSB0'
    _baudrate = 38400
    _timeout = 0.3 # serial port buffer will wait until timeout and return result.
    last = time.time()
    comport = serial.Serial(port = _port, baudrate=_baudrate, timeout=_timeout)
    while True:
        last = time.time()
        MESSAGE[1] = MESSAGE[1] + Speed
        MESSAGE[2] = MESSAGE[2] + Speed + 1 
        if MESSAGE[1] % 10 == 0:
            comport.write(bytearray([3, 0, 0]))
            packets = map(ord, comport.read(9))
            print "Encoder: ", packets
        if MESSAGE[1] == 255:
            Speed = -1
        elif MESSAGE[1] == 127:
            Speed = 1
        else:
            comport.write(MESSAGE)
            packets = map(ord, comport.read(2))
            print "Speed Exec: ", packets

        time.sleep(0.1)
    comport.close()
