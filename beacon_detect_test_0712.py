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
finding_beacon = False
finding_beacon_status = False
beacon_data = [0, 0, 0, 0]
distance = [100, 100, 100, 100, 100, 100, 100, 100]

lighthouse = serial.Serial(port='/dev/rfcomm1', baudrate=115200, timeout=0.5)

ports = list(serial.tools.list_ports.comports())
if len(ports) == 0:
    print('Cannot find any serial ports: check MBED connection.')
    quit()
# Configure the serial connection to the MBED
mbed_of_BadBoy = serial.Serial(
    port=str(ports[1][0]),
    baudrate=115200,
    timeout=0
)
swarm_robot = serial.Serial(
  port=str(ports[0][0]),
  baudrate=115200,
  timeout=0
)

port0_SNR = ports[0][2]
port1_SNR = ports[1][2]
print(port0_SNR[-5:])
print(port1_SNR[-5:])
if port0_SNR[-5:] == '27CE1' and port1_SNR[-5:] == '1B169':
    mbed_of_BadBoy = serial.Serial(
        port=str(ports[0][0]),
        baudrate=115200,
        timeout=0
    )
    swarm_robot = serial.Serial(
      port=str(ports[1][0]),
      baudrate=115200,
      timeout=0
    )
    print('badboy mbed', ports[0])
elif port0_SNR[-5:] == '1B169' and port1_SNR[-5:] == '27CE1':
    swarm_robot = serial.Serial(
        port=str(ports[0][0]),
        baudrate=115200,
        timeout=0
    )
    mbed_of_BadBoy = serial.Serial(
        port=str(ports[1][0]),
        baudrate=115200,
        timeout=0
    )
    print('badboy mbed', ports[1])


def decode_2y0a21(int_val):
    # Decode a int value [0-255] corresponding to 0-3.3V for the Sharp GP2Y0A21 sensor
    # Use values from datasheet for the 8.75cm to 80cm range
    # Power curve is a good fit
    # y = 4336.6 * x ^ -1.165579 (where x = 77.3 x voltage)
    number = float(int_val)
    if number < 29.0:
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
    if number < 27.0:
        distance = 30
    else:
        x_inv_power = pow(number, -1.02475)
        distance = x_inv_power * 1093.955
    return round(distance,1)


def read():
    global running
    global finding_beacon
    global beacon_data
    global distance
    beacon_raw_data = [0, 0, 0, 0]
    while running:
        mbed_data = mbed_of_BadBoy.read(32)
        if len(mbed_data) > 0:
            if mbed_data[0] == '\x1B' and len(mbed_data) == 9:
                print("mbed_data")
                distance[0] = decode_2y0a41(struct.unpack('B', mbed_data[1])[0])
                distance[1] = decode_2y0a21(struct.unpack('B', mbed_data[2])[0])
                distance[2] = decode_2y0a41(struct.unpack('B', mbed_data[3])[0])
                distance[3] = decode_2y0a41(struct.unpack('B', mbed_data[4])[0])
                distance[4] = decode_2y0a21(struct.unpack('B', mbed_data[5])[0])
                distance[5] = decode_2y0a41(struct.unpack('B', mbed_data[6])[0])
                distance[6] = decode_2y0a41(struct.unpack('B', mbed_data[7])[0])
                distance[7] = decode_2y0a41(struct.unpack('B', mbed_data[8])[0])
                print(distance)
        sleep(0.05)
        swarm_data = swarm_robot.read(32)
        if len(swarm_data) > 0:
            if swarm_data[0] == '\x1A':
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
                if abs(beacon_raw_data[0] - beacon_raw_data[2]) == 1 or abs(beacon_raw_data[0] - beacon_raw_data[2]) == 7:
                    if beacon_raw_data[1] > 50 and beacon_raw_data[3] > 50:
                        beacon_data = beacon_raw_data
                        finding_beacon = True
                        lighthouse.write(b'\x1d\x00\x01\x00\x1d')
                        print('stop turning')
                    else:
                        beacon_data = [0, 0, 0, 0]
                        finding_beacon = False
                        lighthouse.write(b'\x1d\x01\x00\x00\x1d')
                        print('start turning')
                else:
                    beacon_data = [0, 0, 0, 0]
                    finding_beacon = False
                    lighthouse.write(b'\x1d\x01\x00\x00\x1d')
                    print('start turning')
                print('finding beacon = ', finding_beacon)
                print(beacon_data)
        sleep(0.05)


def poll():

    global running
    get_beacon = b'\x1d\x7d\x01\x01\x1d'
    get_BadBoy_ir_sensors = b'\x1d\x6b\x01\x01\x1d'
    while running:
        swarm_robot.write(get_beacon)
        sleep(1)
        mbed_of_BadBoy.write(get_BadBoy_ir_sensors)
        sleep(1)


def brake():
    message = b'\x1d\x06\x00\x00\x1d'
    mbed_of_BadBoy.write(message)
    print('BRAKE MOTORS')


def coast():
    message = b'\x1d\x07\x00\x00\x1d'
    mbed_of_BadBoy.write(message)
    print('COAST MOTORS')


def backwards(speed):
    message = b'\x1d\x03'
    kspeed = speed * 32767.0
    byte0 = int(kspeed / 256)
    byte1 = int(kspeed % 256)
    message += chr(byte0)
    message += chr(byte1)
    message += chr(29)
    mbed_of_BadBoy.write(message)
    print('BACKWARDS')


def forwards(speed):
    message = b'\x1d\x03'
    kspeed = speed * 32767.0
    byte0 = int(kspeed / 256) + 128
    byte1 = int(kspeed % 256)
    message += chr(byte0)
    message += chr(byte1)
    message += chr(29)
    mbed_of_BadBoy.write(message)
    print('FORWARDS')


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
    #print('SET LEFT SPEED', speed)


def set_right_speed(speed):
    message = b'\x1d\x02'
    kspeed = speed * 32767.0
    byte0 = int(kspeed / 256) + 128
    byte1 = int(kspeed % 256)
    message += chr(byte0)
    message += chr(byte1)
    message += chr(29)
    mbed_of_BadBoy.write(message)


def motor():
    global distance
    global running
    while running:
        if distance[1] < 15:
            print("turn right")
            set_right_speed(0.4)
            set_left_speed(0.7)

        elif distance[3] < 15:
            print("turn left")
            set_right_speed(0.8)
            set_left_speed(0.3)
        else:
            print("forward")
            set_right_speed(1)
            set_left_speed(0.956)
        sleep(1)


def avoid_obstacles():
    global running
    global swarm_robot
    global mbed_of_BadBoy
    if swarm_robot.isOpen():
        swarm_robot.close()
    if mbed_of_BadBoy.isOpen():
        mbed_of_BadBoy.close()
    if lighthouse.isOpen():
        lighthouse.close()
    swarm_robot.open()
    mbed_of_BadBoy.open()
    lighthouse.open()

    port_correct_connect = 0
    while port_correct_connect == 0:
        mbed_of_BadBoy.write(b'\x1d\xff\x00\x00\x1d')
        swarm_robot.write(b'\x1d\xff\x00\x00\x1d')
        sleep(0.1)
        badboy_confirm = mbed_of_BadBoy.readline()
        swarm_confirm = swarm_robot.readline()
        print("badboy =", badboy_confirm)
        print("swarm =", swarm_confirm)
        if swarm_confirm == "badboy" and badboy_confirm == "swarm":
            temp = mbed_of_BadBoy
            mbed_of_BadBoy = swarm_robot
            swarm_robot = temp
            port_correct_connect = 1
            print("exchange correct")
        elif badboy_confirm == "badboy" and swarm_confirm == "swarm":
            port_correct_connect = 1
            print("correct")
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
    swarm_robot.close()
    mbed_of_BadBoy.close()
    lighthouse.close()

print("start")
avoid_obstacles()
print("end")



