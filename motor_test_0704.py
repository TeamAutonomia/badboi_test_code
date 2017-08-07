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
    port=str(ports[1][0]),
    baudrate=115200,
    timeout=0
)
average_distance = [0, 0, 0, 0, 0, 0, 0, 0]


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
    global average_distance
    distance = [0, 0, 0, 0, 0, 0, 0, 0]
    count = 0
    while running:
        data = ser.read(32)
        if len(data) > 0:
            message_understood = False
            if data[0] == '\x1B' and len(data) == 9:
                if count < 10:
                    count += 1
                    print("read IR sensor")

                    distance[0] += decode_2y0a41(struct.unpack('B', data[1])[0])
                    distance[1] += decode_2y0a21(struct.unpack('B', data[2])[0])
                    distance[2] += decode_2y0a41(struct.unpack('B', data[3])[0])
                    distance[3] += decode_2y0a41(struct.unpack('B', data[4])[0])
                    distance[4] += decode_2y0a21(struct.unpack('B', data[5])[0])
                    distance[5] += decode_2y0a41(struct.unpack('B', data[6])[0])
                    distance[6] += decode_2y0a41(struct.unpack('B', data[7])[0])
                    distance[7] += decode_2y0a41(struct.unpack('B', data[8])[0])
                    message_understood = True
                else:
                    for i in range(0, 8):
                        average_distance[i] = distance[i] // 10
                        distance[i] = 0
                    print("aver:", average_distance)
                    motor()
                    count = 0
            if data[0] == '\x1E':
                #print 'Ack Message Received:',data.encode('hex')
                message_understood = True
            if data[0] == '\x1F':
                #print 'Response Message Received:',data.encode('hex')
                message_understood = True
            #if not message_understood:
                #print 'Unknown Message Received:',data.encode('hex')
        sleep(0.01)


def poll():
    get_sensors = b'\x1d\x6b\x01\x01\x1d'
    while running:
        ser.write(get_sensors)
        sleep(0.05)


def brake():
    message = b'\x1d\x06\x00\x00\x1d'
    ser.write(message)
    print('BRAKE MOTORS')


def coast():
    message = b'\x1d\x07\x00\x00\x1d'
    ser.write(message)
    print('COAST MOTORS')


def backwards(speed):
    message = b'\x1d\x03'
    kspeed = speed * 32767.0
    byte0 = int(kspeed / 256)
    byte1 = int(kspeed % 256)
    message += chr(byte0)
    message += chr(byte1)
    message += chr(29)
    ser.write(message)
    print('BACKWARDS')


def forwards(speed):
    message = b'\x1d\x03'
    kspeed = speed * 32767.0
    byte0 = int(kspeed / 256) + 128
    byte1 = int(kspeed % 256)
    message += chr(byte0)
    message += chr(byte1)
    message += chr(29)
    ser.write(message)
    print('FORWARDS')


def right_turn(speed):
    message = b'\x1d\x08'
    kspeed = speed * 32767.0
    byte0 = int(kspeed / 256) + 128
    byte1 = int(kspeed % 256)
    message += chr(byte0)
    message += chr(byte1)
    message += chr(29)
    ser.write(message)
    print('TURN RIGHT')


def left_turn(speed):
    message = b'\x1d\x08'
    kspeed = speed * 32767.0
    byte0 = int(kspeed / 256)
    byte1 = int(kspeed % 256)
    message += chr(byte0)
    message += chr(byte1)
    message += chr(29)
    ser.write(message)
    print('TURN LEFT')


def set_left_speed(speed):
    message = b'\x1d\x01'
    kspeed = speed * 32767.0
    byte0 = int(kspeed / 256) + 128
    byte1 = int(kspeed % 256)
    message += chr(byte0)
    message += chr(byte1)
    message += chr(29)
    ser.write(message)
    #print('SET LEFT SPEED', speed)


def set_right_speed(speed):
    message = b'\x1d\x02'
    kspeed = speed * 32767.0
    byte0 = int(kspeed / 256) + 128
    byte1 = int(kspeed % 256)
    message += chr(byte0)
    message += chr(byte1)
    message += chr(29)
    ser.write(message)
    #print('SET RIGHT SPEED', speed)


def motor():
    global average_distance
    #if average_distance[2] < 10 or average_distance[3] < 10:
    # print("center")
    #if average_distance[2] < average_distance[3]:
    #   set_right_speed(0.3)
    # set_left_speed(0.7)
    #else:
    # set_right_speed(0.7)
    #set_left_speed(0.3)
    if average_distance[1] < 15:
        #print("obstacle on the left side")
        print("turn right")
        set_right_speed(0.4)
        set_left_speed(0.7)

    elif average_distance[3] < 15:
        #print("obstacle on the right side")
        print("turn left")
        set_right_speed(0.8)
        set_left_speed(0.3)
    else:
        print("forward")
        set_right_speed(1)
        set_left_speed(0.956)


def avoid_obstacles():
    global running
    if ser.isOpen():
        ser.close()
    ser.open()
    poll_thread = threading.Thread(target=poll)
    poll_thread.start()
    read_thread = threading.Thread(target=read)
    read_thread.start()
    motor_thread = threading.Thread(target=motor)
    motor_thread.start()

    sleep(10)
    running = False
    brake()
    sleep(1)
    ser.close()

print("start")
avoid_obstacles()
print("end")
