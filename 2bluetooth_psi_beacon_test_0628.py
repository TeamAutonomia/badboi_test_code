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

blue_tooth_beacon = serial.Serial(port='/dev/rfcomm1', baudrate=115200, timeout=0.5)
blue_tooth_psi = serial.Serial(port='/dev/rfcomm2', baudrate=115200, timeout=0.5)


def read():
    beacon_data = [0, 0, 0, 0]
    while running:
        data_beacon = blue_tooth_beacon.read(32)
        data_psi = blue_tooth_psi.read()
        if len(data_beacon) > 0:
            print("beacon")
            print(data_beacon)
        if len(data_psi) > 0:
            print("psi")
            i = 1
            j = 0
            start = 1
            while i < len(data_psi):
                if data_psi[i] == ",":
                    end = i
                    beacon_data[j] = data_psi[start: end]
                    beacon_data[j] = int(beacon_data[j])
                    start = i + 1
                    j += 1
                i += 1
            print(beacon_data)
        sleep(0.05)


def poll():
    test_beacon = b'\x1d\x01\x01\x01\x1d'
    get_bg_ir_sensors = b'\x1d\x5c\x01\x01\x1d'
    while running:
        blue_tooth_beacon.write(test_beacon)
        sleep(0.5)
        blue_tooth_psi.write(get_bg_ir_sensors)
        sleep(0.5)


def avoid_obstacles():
    global running
    if blue_tooth_beacon.isOpen():
        blue_tooth_beacon.close()
    blue_tooth_beacon.open()
    if blue_tooth_psi.isOpen():
        blue_tooth_psi.close()
    blue_tooth_psi.open()

    poll_thread = threading.Thread(target=poll)
    poll_thread.start()
    read_thread = threading.Thread(target=read)
    read_thread.start()
    sleep(10)
    running = False
    sleep(1)
    blue_tooth_beacon.close()
    blue_tooth_psi.close()

print("start")
avoid_obstacles()
print("end")
