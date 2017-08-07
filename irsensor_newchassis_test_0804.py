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
distance = [80, 80, 80, 80, 80, 30, 30]
beacon_data = [0, 0, 0, 0]
ports = list(serial.tools.list_ports.comports())
if len(ports) == 0:
    print('Cannot find any serial ports: check MBED connection.')
    quit()
# Configure the serial connection to the MBED
mbed_of_BadBoy = serial.Serial(
    port=str(ports[0][0]),
    baudrate=115200,
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
    global distance
    while running:
        data = mbed_of_BadBoy.read(32)
        if len(data) > 0:
            if data[0] == '\x1B' and len(data) == 9:
                distance[0] = decode_2y0a21(struct.unpack('B', data[1])[0])
                distance[1] = decode_2y0a21(struct.unpack('B', data[2])[0])
                distance[2] = decode_2y0a21(struct.unpack('B', data[3])[0])
                distance[3] = decode_2y0a21(struct.unpack('B', data[4])[0])
                distance[4] = decode_2y0a21(struct.unpack('B', data[5])[0])
                distance[5] = decode_2y0a41(struct.unpack('B', data[6])[0])
                distance[6] = decode_2y0a41(struct.unpack('B', data[8])[0])
                print('distance:', distance)
        sleep(0.05)


def poll():
    get_sensors = b'\x1d\x6b\x01\x01\x1d'
    while running:
        mbed_of_BadBoy.write(get_sensors)
        sleep(0.5)


def brake():
    message = b'\x1d\x06\x00\x00\x1d'
    mbed_of_BadBoy.write(message)
    print('BRAKE MOTORS')



def right_turn(speed):
    message = b'\x1d\x08'
    kspeed = speed * 32767.0
    byte0 = int(kspeed / 256) + 128
    byte1 = int(kspeed % 256)
    message += chr(byte0)
    message += chr(byte1)
    message += chr(29)
    mbed_of_BadBoy.write(message)
    print('TURN RIGHT')


def left_turn(speed):
    message = b'\x1d\x08'
    kspeed = speed * 32767.0
    byte0 = int(kspeed / 256)
    byte1 = int(kspeed % 256)
    message += chr(byte0)
    message += chr(byte1)
    message += chr(29)
    mbed_of_BadBoy.write(message)
    print('TURN LEFT')


def set_left_speed(speed):
        message = b'\x1d\x01'
        kspeed = speed * 32767.0
        byte0 = int(kspeed / 256) + 128
        byte1 = int(kspeed % 256)
        message += chr(byte0)
        message += chr(byte1)
        message += chr(29)
        mbed_of_BadBoy.write(message)
        # print('SET LEFT SPEED', speed)


def set_right_speed(speed):
        message = b'\x1d\x02'
        kspeed = speed * 32767.0
        byte0 = int(kspeed / 256) + 128
        byte1 = int(kspeed % 256)
        message += chr(byte0)
        message += chr(byte1)
        message += chr(29)
        mbed_of_BadBoy.write(message)


def turn_to_beacon():
    global beacon_data
    if abs(beacon_data[0] - beacon_data[2]) == 1:
        if 7 > beacon_data[0] + beacon_data[2] > 0:
            print("turn right beacon")
            set_right_speed(0.4)
            set_left_speed(0.7)
        elif beacon_data[0] + beacon_data[2] > 7:
            print("turn left beacon")
            set_right_speed(0.8)
            set_left_speed(0.3)
        elif beacon_data[0] + beacon_data[2] == 7:
            print("turn around beacon")
            set_right_speed(1)
            set_left_speed(-0.956)
        else:
            print("forward")
            set_right_speed(1)
            set_left_speed(0.956)


# according to the IR sensor data, make the badboy move forwards or turn.
def motor():
    global distance
    global running
    #global beacon_data
    while running:
        if distance[2] < 40:
            if distance[2] < 20:
                if distance[1] < distance[3]:
                    right_turn(1)
                else:
                    left_turn(1)
            else:
                if distance[1] < 20 or distance[3] < 20:
                    if distance[1] < distance[3]:
                        print("turn right")
                        set_right_speed(0.4)
                        set_left_speed(0.7)
                    else:
                        print("turn left")
                        set_right_speed(0.8)
                        set_left_speed(0.3)
                else:
                    turn_to_beacon()
        elif distance[1] < 40 or distance[0] < 40:
            if distance[1]<20 or distance[0] < 20:
                right_turn(1)
            else:
                print("turn right")
                set_right_speed(0.4)
                set_left_speed(0.7)
        elif distance[3] < 40 or distance[4] < 40:
            if distance[3] < 20 or distance[4] < 20:
                left_turn(1)
            else:
                print("turn left")
                set_right_speed(0.8)
                set_left_speed(0.3)
        else:
            turn_to_beacon()
        sleep(0.1)


def avoid_obstacles():
    global running
    if mbed_of_BadBoy.isOpen():
        mbed_of_BadBoy.close()
        mbed_of_BadBoy.open()
    poll_thread = threading.Thread(target=poll)
    poll_thread.start()
    read_thread = threading.Thread(target=read)
    read_thread.start()
    motor_thread = threading.Thread(target=motor)
    motor_thread.start()
    sleep(60)
    running = False
    brake()
    sleep(1)
    mbed_of_BadBoy.close()

print("start")
avoid_obstacles()
print("end")
