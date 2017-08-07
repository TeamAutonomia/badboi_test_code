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
ports = list(serial.tools.list_ports.comports())
if len(ports) == 0:
    print('Cannot find any BlueTooth ports: check MBED connection.')
    quit()
# Configure the bluetooth connection to the MBED
blue_tooth = serial.Serial(
    port="/dev/rfcomm0",
    baudrate=115200,
    timeout=0
)


'''
def read():
    while running:
        data = blue_tooth.read(32)
        if len(data) > 0:
            print("badboy")
            message_understood = False
            if data[0] == '\x1B' and len(data) == 9:
                print('1=', struct.unpack('B',data[1])[0])
                print('2=', struct.unpack('B',data[2])[0])
                print('3=', struct.unpack('B',data[3])[0])
                print('4=', struct.unpack('B',data[4])[0])
                print('5=', struct.unpack('B',data[5])[0])
                print('6=', struct.unpack('B',data[6])[0])
                print('7=', struct.unpack('B',data[7])[0])
                print('8=', struct.unpack('B',data[8])[0])
                message_understood = True

        sleep(0.05)


def poll():
    get_bg_ir_sensors = b'\x1d\x5c\x01\x01\x1d'
    while running:
        blue_tooth.write(get_bg_ir_sensors)
        sleep(0.5)


def avoid_obstacles():
    global running
    if blue_tooth.isOpen():
        blue_tooth.close()
    blue_tooth.open()
    poll_thread = threading.Thread(target=poll)
    poll_thread.start()
    read_thread = threading.Thread(target=read)
    read_thread.start()
    sleep(10)
    running = False
    blue_tooth.close()
'''
print("start")
if blue_tooth.isOpen():
    blue_tooth.close()
blue_tooth.open()
sleep(10)
'''avoid_obstacles()'''
print("end")
