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

# Configure the serial connection to the MBED
# Configure the serial connection to the MBED
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

#lighthouse = serial.Serial(port='/dev/rfcomm1', baudrate=115200, timeout=0.5)

obstacles_distance = [0, 0, 0, 0, 0, 0, 0, 0]
beacon_data = [0, 0, 0, 0]
finding_beacon = 0
finding_beacon_status = 0


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


def read_badboy():
    global obstacles_distance
    while running:
        data = mbed_of_BadBoy.read(32)
        if len(data) > 0:
            message_understood = False
            if data[0] == '\x1B' and len(data) == 9:
                print(running)
                obstacles_distance[0] += decode_2y0a41(struct.unpack('B', data[1])[0])
                obstacles_distance[1] += decode_2y0a21(struct.unpack('B', data[2])[0])
                obstacles_distance[2] += decode_2y0a41(struct.unpack('B', data[3])[0])
                obstacles_distance[3] += decode_2y0a41(struct.unpack('B', data[4])[0])
                obstacles_distance[4] += decode_2y0a21(struct.unpack('B', data[5])[0])
                obstacles_distance[5] += decode_2y0a41(struct.unpack('B', data[6])[0])
                obstacles_distance[6] += decode_2y0a41(struct.unpack('B', data[7])[0])
                obstacles_distance[7] += decode_2y0a41(struct.unpack('B', data[8])[0])
                message_understood = True
            if data[0] == '\x1E':
                #print 'Ack Message Received:',data.encode('hex')
                message_understood = True
            if data[0] == '\x1F':
                #print 'Response Message Received:',data.encode('hex')
                message_understood = True
            #if not message_understood:
                #print 'Unknown Message Received:',data.encode('hex')
        sleep(0.01)


def read_beacon():
    global beacon_data
    global finding_beacon
    data = [0, 0, 0, 0]
    while running:
        swarm_data = swarm_robot.read(32)
        if len(swarm_data) > 0:
            if swarm_data[0] == '\x1A':
                print("swarm_data")
                # message_understood = False
                i = 1
                j = 0
                start = 1
                while i < len(swarm_data):
                    if swarm_data[i] == ",":
                        end = i
                        data[j] = swarm_data[start: end]
                        data[j] = int(beacon_data[j])
                        start = i + 1
                        j += 1
                    i += 1
                if abs(data[0] - data[3]) == 1 or abs(data[0] - data[3]) == 7:
                    if data[1] > 50 and data[3] > 50:
                        beacon_data = data
                        finding_beacon = 1
                    else:
                        beacon_data = [0, 0, 0, 0]
                        finding_beacon = 0
                else:
                    beacon_data = [0, 0, 0, 0]
                    finding_beacon = 0
                print('finding beacon = ', finding_beacon)
                print(beacon_data)
                # message_understood = True
        sleep(0.05)


'''def write_to_lighthouse():
    global finding_beacon_status
    stop_turning = b'\x1d\x00\x01\x00\x1d'
    start_turning = b'\x1d\x01\x00\x00\x1d'

    if finding_beacon_status != finding_beacon:
        if finding_beacon == 1:
            lighthouse.write(stop_turning)
        elif finding_beacon == 0:
            lighthouse.write(start_turning)
        finding_beacon_status = finding_beacon
    sleep(0.05)
'''


def poll():

    get_badboy_sensors = b'\x1d\x6b\x01\x01\x1d'
    #get_beacon = b'\x1d\x7d\x01\x01\x1d'
    while running:
        mbed_of_BadBoy.write(get_badboy_sensors)
        sleep(0.05)
        #swarm_robot.write(get_beacon)
        sleep(0.05)

'''
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
    #print('SET RIGHT SPEED', speed)


def motor():
    while running:
        if obstacles_distance[1] < 15:
            #print("obstacle on the left side")
            print("turn right")
            set_right_speed(0.4)
            set_left_speed(0.7)

        elif obstacles_distance[3] < 15:
            #print("obstacle on the right side")
            print("turn left")
            set_right_speed(0.8)
            set_left_speed(0.3)
        else:
            print("forward")
            set_right_speed(1)
            set_left_speed(0.956)
        sleep(0.05)
'''

def avoid_obstacles():
    global swarm_robot
    global mbed_of_BadBoy
    global running
    if mbed_of_BadBoy.isOpen():
        mbed_of_BadBoy.close()
    mbed_of_BadBoy.open()
    if swarm_robot.isOpen():
        swarm_robot.close()
    swarm_robot.open()
    '''
    if lighthouse.isOpen():
        lighthouse.close()
    lighthouse.open()'''
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
    read_badboy_thread = threading.Thread(target=read_badboy())
    read_badboy_thread.start()
    read_beacon_thread = threading.Thread(target=read_beacon())
    read_beacon_thread.start()
    '''motor_thread = threading.Thread(target=motor)
    motor_thread.start()
    write_to_lighthouse_thread = threading.Thread(target=write_to_lighthouse())
    write_to_lighthouse_thread.start()'''
    sleep(10)
    running = False
    print('test', running)
    ''' brake()'''
    sleep(1)
    mbed_of_BadBoy.close()
    swarm_robot.close()
    #lighthouse.close()

print("start")
avoid_obstacles()
print("end")
