import serial
import threading
import struct
import serial.tools.list_ports
# import getch
import sys, getopt
import os.path
import time
import math
from time import sleep

ports = list(serial.tools.list_ports.comports())
print(ports)
swarm_robot = serial.Serial(
  port=str(ports[0][0]),
  baudrate=115200,
  timeout=0
)
get_beacon = b'\x1d\x7d\x01\x01\x1d'
if swarm_robot.isOpen():
    swarm_robot.close()
swarm_robot.open()
swarm_robot.write(get_beacon)
beacon_raw_data = [0, 0, 0, 0]
while 1:
    swarm_data = swarm_robot.read(32)
    if len(swarm_data) > 0:
        if swarm_data[0] == '\x1F':
            print("swarm_data")
            i = 1
            j = 0
            start = 1
            while i < len(swarm_data):
                if swarm_data[i] == ",":
                    end = i
                    beacon_raw_data[j] = swarm_data[start: end]
                    beacon_raw_data[j] = int(beacon_raw_data[j])
                    start = i + 1
                    j += 1
                i += 1
            print(beacon_raw_data)
