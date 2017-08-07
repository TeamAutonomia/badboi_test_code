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
    print('Cannot find any serial ports: check MBED connection.')
    quit()
# Configure the serial connection to the MBED
ser = serial.Serial(
    port=str(ports[0][0]),
    baudrate=460800,
    timeout=0
)


def decode_2y0a21(int_val):
    # Decode a int value [0-255] corresponding to 0-3.3V for the Sharp GP2Y0A21 sensor
    # Use values from datasheet for the 8.75cm to 80cm range
    # Power curve is a good fit
    # y = 4336.6 * x ^ -1.165579 (where x = 77.3 x voltage)
    number = float(int_val)
    if number < 32.0:
        distance = 80
    else:
        x_inv_power = pow(number, -1.165579)
        distance = x_inv_power * 4336.6
    return round(distance,1)


def decode_2y0a41(int_val):
    # Decode a int value [0-255] corresponding to 0-3.3V for the Sharp GP2Y0A41 sensor
    # Use values from datasheet for the 3.5cm to 30cm range
    # Power curve is a good fit
    # y = 1093.955 * x ^ -1.02475 (where x = 77.3 x voltage)
    number = float(int_val)
    if number < 35.0:
        distance = 30
    else:
        x_inv_power = pow(number, -1.02475)
        distance = x_inv_power * 1093.955
    return round(distance,1)


def read():
    while running:
        data = ser.read(32)
        if len(data) > 0:
            message_understood = False
            if data[0] == '\x1B' and len(data) == 9:
                print('1=', decode_2y0a41(struct.unpack('B',data[1])[0]))
                print('2=', decode_2y0a21(struct.unpack('B',data[2])[0]))
                print('3=', decode_2y0a41(struct.unpack('B',data[3])[0]))
                print('4=', decode_2y0a41(struct.unpack('B',data[4])[0]))
                print('5=', decode_2y0a21(struct.unpack('B',data[5])[0]))
                print('6=', decode_2y0a41(struct.unpack('B',data[6])[0]))
                print('7=', decode_2y0a41(struct.unpack('B',data[7])[0]))
                print('8=', decode_2y0a41(struct.unpack('B',data[8])[0]))
                '''print('1=', decode_2y0a41(struct.unpack('B', data[1])[0]),'2=', decode_2y0a21(struct.unpack('B',data[2])[0]), '3=', decode_2y0a41(struct.unpack('B',data[3])[0]), '4=', decode_2y0a41(struct.unpack('B',data[4])[0]), '5=', decode_2y0a21(struct.unpack('B',data[5])[0]), '6=', decode_2y0a41(struct.unpack('B',data[6])[0]), '7=', decode_2y0a41(struct.unpack('B',data[7])[0]), '8=', decode_2y0a41(struct.unpack('B',data[8])[0]))
                message_understood = True'''
            if data[0] == '\x1E':
                #print 'Ack Message Received:',data.encode('hex')
                message_understood = True
            if data[0] == '\x1F':
                #print 'Response Message Received:',data.encode('hex')
                message_understood = True
            #if not message_understood:
                #print 'Unknown Message Received:',data.encode('hex')
        sleep(0.05)


def poll():
    get_sensors = b'\x1d\x6b\x01\x01\x1d'
    while running:
        ser.write(get_sensors)
        sleep(0.5)


def avoid_obstacles():
    global running
    if ser.isOpen():
        ser.close()
    ser.open()
    poll_thread = threading.Thread(target=poll)
    poll_thread.start()
    read_thread = threading.Thread(target=read)
    read_thread.start()

    sleep(60)

    #getch.getch()
    running = False
    ser.close()

print("start")
avoid_obstacles()
print("end")
