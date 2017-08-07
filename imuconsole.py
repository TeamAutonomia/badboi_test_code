import curses
import serial
import threading
import struct
import serial.tools.list_ports
import sys, getopt
import RTIMU
import os.path
import time
import math
import psutil
from time import sleep

altitude_offset = 0.0
roll_offset = 0.0
pitch_offset = 0.0
yaw_offset = 0.0
current_altitude = 0.0
current_roll = 0.0
current_pitch = 0.0
current_yaw = 0.0
running=True
SETTINGS_FILE = "RTIMULib"
if not os.path.exists(SETTINGS_FILE + ".ini"):
   print("Settings file does not exist, will be created")
s = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(s)
pressure = RTIMU.RTPressure(s)
if (not imu.IMUInit()):
   print("IMU Init Failed")
   sys.exit(1)
imu.setSlerpPower(0.02)
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)
if (not pressure.pressureInit()):
   print("Pressure sensor Init Failed")
poll_interval = imu.IMUGetPollInterval()

def computeHeight(pressure):
    return 44330.8 * (1 - pow(pressure / 1013.25, 0.190263));
    
def read_imu ():
  global running
  global current_roll
  global current_pitch
  global current_yaw
  global current_altitude
  while running:
    if imu.IMURead():
        # x, y, z = imu.getFusionData()
        # print("%f %f %f" % (x,y,z))
        data = imu.getIMUData()
        (data["pressureValid"], data["pressure"], data["temperatureValid"], data["temperature"]) = pressure.pressureRead()
        fusionPose = data["fusionPose"]
        current_roll = math.degrees(fusionPose[0])
        current_pitch = math.degrees(fusionPose[1])
        current_yaw = math.degrees(fusionPose[2])
        roll =  current_roll - roll_offset
        pitch = current_pitch - pitch_offset
        yaw = current_yaw - yaw_offset
        r_string = '{0:.1f}'.format(roll)
        while len(r_string)<6: r_string = ' '+r_string
        p_string = '{0:.1f}'.format(pitch)
        while len(p_string)<6: p_string = ' '+p_string
        y_string = '{0:.1f}'.format(yaw)
        while len(y_string)<6: y_string = ' '+y_string        
	imubox.addstr(1,22,r_string,curses.A_BOLD)
        imubox.addstr(1,44,p_string,curses.A_BOLD)
        imubox.addstr(1,67,y_string,curses.A_BOLD)
        if (data["pressureValid"]):
            ps_string = '{0:.1f}'.format(data["pressure"])
            while len(ps_string)<6: ps_string = ' '+ps_string       
            current_altitude = computeHeight(data["pressure"])
            altitude = current_altitude - altitude_offset 
            hg_string = '{0:.1f}'.format(altitude)
            while len(hg_string)<6: hg_string = ' '+hg_string        
            imubox.addstr(2,22,ps_string,curses.A_BOLD)
            imubox.addstr(2,44,hg_string,curses.A_BOLD)
        if (data["temperatureValid"]):
            t_string = '{0:.1f}'.format(data["temperature"])
            while len(t_string)<6: t_string = ' '+t_string        
            imubox.addstr(2,67,t_string,curses.A_BOLD)
    #imubox.refresh()
    time.sleep(poll_interval*1.0/1000.0)

def decode_2y0a21( int_val ):
    # Decode a int value [0-255] corresponding to 0-3.3V for the Sharp GP2Y0A21 sensor
    # Use values from datasheet for the 8.75cm to 80cm range
    # Power curve is a good fit
    # y = 4336.6 * x ^ -1.165579 (where x = 77.3 x voltage)
    number = float( int_val )
    if(number < 35.0) :
	ret_string = '----'
    else :   
    	x_inv_power = pow(number, -1.165579)
    	distance = x_inv_power * 4336.6
    	ret_string = '{0:.1f}'.format(distance)
        while(len(ret_string) < 4) : ret_string = ' ' + ret_string
    return ret_string

def decode_2y0a41( int_val ):
    # Decode a int value [0-255] corresponding to 0-3.3V for the Sharp GP2Y0A41 sensor
    # Use values from datasheet for the 3.5cm to 30cm range
    # Power curve is a good fit
    # y = 1093.955 * x ^ -1.02475 (where x = 77.3 x voltage)
    number = float( int_val )
    if(number < 35.0) :
	ret_string = '----'
    else :   
    	x_inv_power = pow(number, -1.02475)
    	distance = x_inv_power * 1093.955
    	ret_string = '{0:.1f}'.format(distance)
        while(len(ret_string) < 4) : ret_string = ' ' + ret_string
    return ret_string

def pistats():
    global running
    while running:
        with open ("/sys/class/thermal/thermal_zone0/temp") as temperature_file:
            data = temperature_file.readline()
            pibox.addstr(1,11,'{0:.1f}'.format(float(data) * 0.001) + ' C  ',curses.A_BOLD)
            cpu_pc = str(psutil.cpu_percent())
            while len(cpu_pc) < 5: cpu_pc = ' ' + cpu_pc
            pibox.addstr(1,28,cpu_pc + ' %',curses.A_BOLD)
            disk_usage = str(psutil.disk_usage('/')).split("percent=",1)[1].split(')')[0]
            while len(disk_usage) < 5: disk_usage = ' ' + disk_usage
            pibox.addstr(1,48,disk_usage + ' %',curses.A_BOLD)
            mem_usage = str(psutil.virtual_memory()).split("percent=",1)[1].split(',')[0]
            while len(mem_usage) < 5: mem_usage = ' ' + mem_usage
            pibox.addstr(1,67,mem_usage + ' %',curses.A_BOLD)
        sleep(0.33)

def read():
    global running
    while running:
        data = ser.read(32)
        if len(data) > 0:
            message_understood = False
            if data[0]==( '\x1B' ) and len(data) == 9:
                irbox.addstr(2,12,str(struct.unpack('B',data[1])[0]) + '  ',curses.A_BOLD)
                irbox.addstr(3,12,decode_2y0a41(struct.unpack('B',data[1])[0]),curses.A_BOLD)
                irbox.addstr(2,20,str(struct.unpack('B',data[2])[0]) + '  ',curses.A_BOLD)
                irbox.addstr(3,20,decode_2y0a21(struct.unpack('B',data[2])[0]),curses.A_BOLD)
                irbox.addstr(2,28,str(struct.unpack('B',data[3])[0]) + '  ',curses.A_BOLD)
                irbox.addstr(3,28,decode_2y0a41(struct.unpack('B',data[3])[0]),curses.A_BOLD)
                irbox.addstr(2,36,str(struct.unpack('B',data[4])[0]) + '  ',curses.A_BOLD)
                irbox.addstr(3,36,decode_2y0a41(struct.unpack('B',data[4])[0]),curses.A_BOLD)
                irbox.addstr(2,44,str(struct.unpack('B',data[5])[0]) + '  ',curses.A_BOLD)
                irbox.addstr(3,44,decode_2y0a21(struct.unpack('B',data[5])[0]),curses.A_BOLD)
                irbox.addstr(2,52,str(struct.unpack('B',data[6])[0]) + '  ',curses.A_BOLD)
                irbox.addstr(3,52,decode_2y0a41(struct.unpack('B',data[6])[0]),curses.A_BOLD)
                irbox.addstr(2,60,str(struct.unpack('B',data[7])[0]) + '  ',curses.A_BOLD)
                irbox.addstr(3,60,decode_2y0a41(struct.unpack('B',data[7])[0]),curses.A_BOLD)
                irbox.addstr(2,68,str(struct.unpack('B',data[8])[0]) + '  ',curses.A_BOLD)
                irbox.addstr(3,68,decode_2y0a41(struct.unpack('B',data[8])[0]),curses.A_BOLD)
                message_understood = True
            if data[0]==( '\x1C' ) and len(data) == 9:
                mbed_uptime = float(struct.unpack('B',data[1])[0]) * 256.0
                mbed_uptime += struct.unpack('B',data[2])[0]
		mbed_uptime /= 10.0
    	        uptime_string = '{0:.1f}'.format(mbed_uptime)
                while(len(uptime_string) < 7) : uptime_string = ' ' + uptime_string
                statusbox.addstr(1,14,uptime_string,curses.A_BOLD)
                battery_voltage = float(struct.unpack('B',data[3])[0]) * 256.0
                battery_voltage += struct.unpack('B',data[4])[0]
                battery_voltage /= 4096.0
    	        voltage_string = '{0:.2f}'.format(battery_voltage)
                while(len(voltage_string) < 7) : voltage_string = ' ' + voltage_string
                statusbox.addstr(1,54,voltage_string,curses.A_BOLD)
                left_motor_speed = float(struct.unpack('B',data[5])[0] - 127) / 127.0
                right_motor_speed = float(struct.unpack('B',data[6])[0] - 127) / 127.0
                left_motor_speed_string = '{0:.2f}'.format(left_motor_speed) + ' '
                right_motor_speed_string = '{0:.2f}'.format(right_motor_speed) + ' '
                statusbox.addstr(2,14,left_motor_speed_string,curses.A_BOLD)
                statusbox.addstr(2,52,right_motor_speed_string,curses.A_BOLD)
                left_motor_current = float(struct.unpack('B',data[7])[0]) / 96.0
                right_motor_current = float(struct.unpack('B',data[8])[0]) / 96.0
                left_motor_current_string = '{0:.3f}'.format(left_motor_current)
                right_motor_current_string = '{0:.3f}'.format(right_motor_current)
                statusbox.addstr(2,24,left_motor_current_string,curses.A_BOLD)
                statusbox.addstr(2,63,right_motor_current_string,curses.A_BOLD)
                #print 'STATUS Message Received:',data.encode('hex')
                message_understood = True
            if data[0]==( '\x1E' ):
                #print 'Ack Message Received:',data.encode('hex')
                message_understood = True
            if data[0]==( '\x1F' ):
                #print 'Response Message Received:',data.encode('hex')
                message_understood = True
            #if not message_understood:
                #print 'Unknown Message Received:',data.encode('hex')
        sleep(0.005)

def poll():
    get_sensors = b'\x1d\x6b\x01\x01\x1d'
    get_status = b'\x1d\x6c\x01\x01\x1d'
    while running:
        ser.write(get_sensors)
	sleep(0.05)
	ser.write(get_status)
	sleep(0.05)

def serialsetup():
    ports = list(serial.tools.list_ports.comports())
    if len(ports) == 0:
	print 'Cannot find any serial ports: check MBED connection.'
        quit()
    #Configure the serial connection to the MBED
    global ser
    ser = serial.Serial (
        port=str(ports[0][0]),
        baudrate=460800,
        timeout=0
    )

def pagesetup():
    global stdscr
    global irbox
    global imubox
    global statusbox
    global keybox
    global pibox
    stdscr = curses.initscr()
    curses.start_color()
    curses.use_default_colors()
    titlebox = curses.newwin(3,76,1,2)
    titlebox.box()
    titlebox.addstr(1,1,'Practical Robotics Robot Python Console      York Robotics Laboratory 2017',curses.A_STANDOUT)
    imubox = curses.newwin(4,76,8,2)
    imubox.box()
    imubox.addstr(1,1,'IMU Data       Roll:                Pitch:                  Yaw: ')
    imubox.addstr(2,1,'           Pressure:             Altitude:          Temperature:')
    irbox = curses.newwin(5,76,12,2)
    irbox.box()
    irbox.addstr(1,1,'IR Sensor  0(L)    1(FL)   2(FL)   3(FR)   4(FR)   5(R)    6(RR)   7(RL)')
    irbox.addstr(2,1,'Raw Value')
    irbox.addstr(3,1,'Distance')
    statusbox = curses.newwin(4,76,4,2)
    statusbox.box()
    statusbox.addstr(1,1,'MBED Uptime:         s              Battery Voltage:         V')
    statusbox.addstr(2,1,'Left Motor :         [       A]     Right Motor :           [       A]')
    pibox = curses.newwin(3, 76, 17,2)
    pibox.box()
    pibox.addstr(1,1,'CPU Temp:         CPU Load:         Disk Usage:         Mem Usage:')
    keybox = curses.newwin(3, 76, 20, 2)
    keybox.box()
    keybox.addstr(1,1,'Key:             Function:')
    #curses.noecho()
    #curses.cbreak()
    curses.curs_set(0)
    stdscr.keypad(1)
    stdscr.box()
    titlebox.immedok(True)
    imubox.immedok(True)
    irbox.immedok(True)
    stdscr.immedok(True)
    pibox.immedok(True)
    keybox.immedok(True)
    statusbox.immedok(True)
    stdscr.refresh()
    titlebox.refresh()
    imubox.refresh()
    statusbox.refresh()
    keybox.refresh()
    pibox.refresh()
    #irbox.refresh()
    #stdscr.refresh()

def close():
    ser.close()
    curses.nocbreak()
    stdscr.keypad(0)
    curses.echo()
    curses.endwin()
    sys.exit(0)

def zero():
    global altitude_offset
    global yaw_offset
    global pitch_offset
    global roll_offset
    altitude_offset = current_altitude
    yaw_offset = current_yaw
    pitch_offset = current_pitch
    roll_offset = current_roll
    mstring = 'ZERO IMU OFFSETS     '
    keybox.addstr(1,30,mstring,curses.A_BOLD)

def brake():
    message = b'\x1d\x06\x00\x00\x1d'
    ser.write(message)
    mstring = 'BRAKE MOTORS         '
    keybox.addstr(1,30,mstring,curses.A_BOLD)

def coast():
    message = b'\x1d\x07\x00\x00\x1d'
    ser.write(message)
    mstring = 'COAST MOTORS         '
    keybox.addstr(1,30,mstring,curses.A_BOLD)

def backwards(speed):
    message = b'\x1d\x03'
    kspeed = speed * 32767.0
    byte0 = int(kspeed / 256)
    byte1 = int(kspeed % 256)
    message += chr(byte0)
    message += chr(byte1)
    message += chr(29)
    ser.write(message)
    mstring = 'BACKWARDS '+ '{0:.2f}'.format(speed) + '      '
    keybox.addstr(1,30,mstring,curses.A_BOLD)

def forwards(speed):
    message = b'\x1d\x03'
    kspeed = speed * 32767.0
    byte0 = int(kspeed / 256) + 128
    byte1 = int(kspeed % 256)
    message += chr(byte0)
    message += chr(byte1)
    message += chr(29)
    ser.write(message)
    mstring = 'FORWARDS '+ '{0:.2f}'.format(speed) + '      '
    keybox.addstr(1,30,mstring,curses.A_BOLD)

def rightturn(speed):
    message = b'\x1d\x08'
    kspeed = speed * 32767.0
    byte0 = int(kspeed / 256) + 128
    byte1 = int(kspeed % 256)
    message += chr(byte0)
    message += chr(byte1)
    message += chr(29)
    ser.write(message)
    mstring = 'TURN '+ '{0:.2f}'.format(speed) + '           '
    keybox.addstr(1,30,mstring,curses.A_BOLD)

def leftturn(speed):
    message = b'\x1d\x08'
    kspeed = speed * 32767.0
    byte0 = int(kspeed / 256)
    byte1 = int(kspeed % 256)
    message += chr(byte0)
    message += chr(byte1)
    message += chr(29)
    ser.write(message)
    mstring = 'TURN -'+ '{0:.2f}'.format(speed) + '          '
    keybox.addstr(1,30,mstring,curses.A_BOLD)

def curses_main(args):
    serialsetup()
    pagesetup()
    #imusetup()
    keycount = 0
    if ser.isOpen(): ser.close()
    ser.open()  
    pi_thread = threading.Thread(target=pistats, args=())
    pi_thread.start()
    read_thread = threading.Thread(target=read, args=())
    read_thread.start()
    imu_thread = threading.Thread(target=read_imu, args=())
    imu_thread.start()
    poll_thread = threading.Thread(target=poll, args=())
    poll_thread.start()
    oldcar = 0
    speed = 0
    while True:
        c = stdscr.getch()
        if c == ord('q'):
            break
        else:
            if(c != oldcar): 
                if c > 31 and c < 128:
                    keybox.addch(1,6,c,curses.A_BOLD)
                    keybox.addstr('        ')     
                else:
                    keybox.addstr(1,6,'[' + hex(c) + ']   ',curses.A_BOLD)
                oldcar = c
                keycount = 0
                if(c == ord('z')): #z = Zero IMU Values
		    zero()
                if(c == 263): #Backspace = coast
                    coast()
                if(c == 10): #Return = brake
                    brake()
                if(c == 258): #Backwards
                    speed = 0.20
                    backwards(speed)
                if(c == 259): #Forwards
                    speed = 0.20 
                    forwards(speed)
                if(c == 260): #LeftTurn
                    speed = 0.20 
                    leftturn(speed)
                if(c == 261): #RightTurn
                    speed = 0.20 
                    rightturn(speed)
            else:
                keycount += 1
                if(c == 258): #Backwards
                    speed += 0.05
                    if(speed > 1): speed=1.0
                    backwards(speed)
		if(c == 259): #Forwards
                    speed += 0.05
                    if(speed > 1): speed=1.0
                    forwards(speed)
		if(c == 260): #LeftTurn
                    speed += 0.05
                    if(speed > 1): speed=1.0
                    leftturn(speed)
		if(c == 261): #RightTurn
                    speed += 0.05
                    if(speed > 1): speed=1.0
                    rightturn(speed)
    running=False
    close()

curses.wrapper(curses_main)
