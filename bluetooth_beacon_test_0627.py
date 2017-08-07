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

running = True

blue_tooth = serial.Serial('/dev/rfcomm0', baudrate=115200, timeout=0.5)

if blue_tooth.isOpen():
    blue_tooth.close()

blue_tooth.open()
i = 0
while 1:
    i += 1
    blue_tooth.write(b'\x1d\x01\x01\x01\x1d')
    sleep(0.5)
    print(blue_tooth.readline())
    sleep(0.5)
    if i == 10:
        break
blue_tooth.close()
