import serial
import time
import numpy as np
import RPi.GPIO as GPIO
import RPi_I2C_driver
import sys

# use $python -m serial.tools.miniterm$ to find the serial ports used
# need to set rules file to attach the ardinos to the correct usb ports
# go to /var/log/messages to find the attaching identifier numbers

sensor1 = '/dev/ttyUSB1'
sensor2 = '/dev/ttyUSB0'

platform1 = '/dev/ttyUSB2'
platform2 = '/dev/ttyUSB3'
platform3 = '/dev/ttyUSB4'

btnPress = False

def button_callback(channel):
    global btnPress
    btnPress = True

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(16, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(16,GPIO.RISING,callback=button_callback)

def getSensorData(ser):
    while True:
        ser.write(b"get value\n")
        time.sleep(1)
        line = ser.readline().decode('utf-8').rstrip()
        if line == "resend":
            continue
        if line == "err":
            continue
        if line != "":
            return float(line)


# Retracts the actuator all the way
def homeActuator(ser):
    ser.write(b"bh")


# Extends the actuator an inch
def calibActuator(ser):
    ser.write(b"bc")


# Send number of steps for actuator to move
def moveActuator(ser, inches):
    steps = int(inches * 11792)
    ser.write(('n' + str(int(steps))).encode('ascii'))


def findThirdAngle(ang1, ang2, distBetween):  # gives the third angle relative to the 1st platform
    pos1 = distBetween * np.sin(ang1 * np.pi / 180)
    pos2 = distBetween * np.sin(ang2 * np.pi / 180)
    d = pos2 - pos1
    ang3 = np.arctan((d / distBetween) * np.pi * 180)
    return ang3


if __name__ == "__main__":
    
    mylcd = RPi_I2C_driver.lcd()

    mylcd.lcd_display_string_pos("Platform Booting", 1, 0)
    mylcd.lcd_display_string_pos("....", 2, 6)

    try:
            # Initialize all serial connections
        sen1 = serial.Serial(sensor1, 9600, timeout=1)
        sen1.flush()

        sen2 = serial.Serial(sensor2, 9600, timeout=1)
        sen2.flush()

        plat1 = serial.Serial(platform1, 9600, timeout=1)
        plat1.flush()

        plat2 = serial.Serial(platform2, 9600, timeout=1)
        plat2.flush()

        plat3 = serial.Serial(platform3, 9600, timeout=1)
        plat3.flush()

        time.sleep(5)
    except Exception as e:
        mylcd.lcd_clear()
        mylcd.lcd_display_string_pos("Error Connecting", 1, 0)
        mylcd.lcd_display_string_pos(str(e)[-13:-1], 2, 0)
        print(e)
        sys.exit()
        
    mylcd.lcd_display_string_pos("Press to Start", 2, 1)
    
#     global btnPress
    while not btnPress:
        time.sleep(0.05)

    # Calibration - Sensors
    mylcd.lcd_clear()
    mylcd.lcd_display_string_pos("Calibrating...", 1, 1)

    print('sensor calib sent')
    sen1.write(b"do calibr\n")
    sen2.write(b"do calibr\n")

    calDone = False
    cal1 = False
    cal2 = False

    while not calDone:
        if sen1.readline().decode('utf-8').rstrip() == 'done':
            cal1 = True
        if sen2.readline().decode('utf-8').rstrip() == 'done':
            cal2 = True
        if cal1 and cal2:
            calDone = True
        time.sleep(.25)
    print('sensor calib recv')

    # Calibration - Actuators
    plats = [plat1, plat2, plat3]
    homed = [False, False, False]
    calibrated = False

    for p in plats:
        homeActuator(p)

    while not all(homed):
        time.sleep(0.5)
        for i in range(len(plats)):
            line = plats[i].readline().decode('utf-8').rstrip()
            if line == 'done':
                homed[i] = True
    print('homed')

    zeroAngle1 = getSensorData(sen1)
    zeroAngle2 = getSensorData(sen2)

    calibActuator(plat2)

    while not calibrated:
        line = plat2.readline().decode('utf-8').rstrip()
        if line == 'done':
            calibrated = True
        time.sleep(0.25)
    print('calibrated')

    calibAngle1 = getSensorData(sen1)
    calibAngle2 = getSensorData(sen2)

    # calib is moving the middle actuator up 1 inch
    perInch1 = zeroAngle1 - calibAngle1 # angle from plat2 to plat1
    perInch2 = zeroAngle2 - calibAngle2 # angle from plat2 to plat3
    perInch3 = (perInch1 + perInch2) / 2  # angle from plat1 to plat3
    
    print(perInch1)
    print(perInch2)
    print(perInch3)

    distBetween = np.abs(1 / np.tan(perInch3 * np.pi / 180))

    print('DISTANCE BETWEEN: ', distBetween)

    homed[1] = False
    homeActuator(plat2)

    while not homed[1]:
        line = plat2.readline().decode('utf-8').rstrip()
        if line == 'done':
            homed[1] = True
        time.sleep(0.25)
    print('homed')
    
    mylcd.lcd_clear()
    mylcd.lcd_display_string_pos("Success", 1, 4)

    # Initialize important values
    MIN_ACTUATOR = 0
    MAX_ACTUATOR = 7.5

    oldAngle1 = getSensorData(sen1)
    oldAngle2 = getSensorData(sen2)
    oldAngle3 = findThirdAngle(oldAngle1, oldAngle2, distBetween)
    oldPos = np.array([0, 0, 0])
    retry = 0
    
    mylcd.lcd_clear()
    mylcd.lcd_display_string_pos("Leveling...", 1, 3)
    mylcd.lcd_display_string_pos("A:{0:.3f}  B:{1:.3f}".format(oldAngle1,oldAngle2), 2, 0)

    btnPress = False

    while True:
#         global btnPress
        if btnPress:
            print('Leveling Aborted')
            mylcd.lcd_display_string_pos("Leveling Aborted", 1, 0)
#             global btnPress
            btnPress = False
            while not btnPress:
                time.sleep(0.05)
            continue
            
        # this uses linear algebra to find the amount to move each actuator quickly
        a = np.array([[perInch3, 0, -perInch3],
                  [perInch3, perInch3, 0],
                  [0, -perInch3, perInch3]])
        
        print('a array', a)
        
        b = np.array([oldAngle1, oldAngle2, oldAngle3])

        print('b array', b)

        newH = np.linalg.solve(a, b)  # array of how far each platform needs to move
        newH = newH - oldPos
        for i in range(len(newH)):
            if newH[i] > 2:
                newH[i] = 2
            if newH[i] < -2:
                newH[i] = -2
        print('Move Amt ', newH)
        newPos = newH + oldPos
        print('Position before check ', newPos)

        if np.amin(newPos) < MIN_ACTUATOR:
            correct_amt = np.abs(np.amin(newPos))
            print('amin: ', correct_amt)
            newPos = newPos + correct_amt
            newH = newH + correct_amt  # corrects for if the result tells platform to move lower than it can
            if np.amax(newPos) > MAX_ACTUATOR:
                retry += 1
                if retry < 5:
                    print('Could not solve for angle... retrying...')
                    continue
                else:
                    print('Angle of ground is too much for platform to handle')
                    break
            retry = 0

        if np.amax(newPos) > MAX_ACTUATOR:
            correct_amt = (np.amax(newPos) - MAX_ACTUATOR)
            newPos = newPos - correct_amt
            newH = newH - correct_amt
            if np.amax(newPos) < MIN_ACTUATOR:
                retry += 1
                if retry < 5:
                    print('Could not solve for angle... retrying...')
                    continue
                else:
                    print('Angle of ground is too much for platform to handle')
                    break
            retry = 0

        print('Positions after check ', newPos)
        moveActuator(plat1, newH[0])
        moveActuator(plat2, newH[1])
        moveActuator(plat3, newH[2])
        
        moving = [True, True, True]
        
        print('moving array before: ', moving)
        print('moving...')
        while any(plat==True for plat in moving):
            time.sleep(0.5)
            for i in range(len(plats)):
                line = plats[i].readline().decode('utf-8').rstrip()
                print('platform ', i, ' line: ', line)
                if line == 'done':
                    moving[i] = False

        print('moving array before: ', moving)
        print('done moving')
        angle1 = getSensorData(sen1)
        angle2 = getSensorData(sen2)
        angle3 = findThirdAngle(angle1, angle2, distBetween)
        
        mylcd.lcd_display_string_pos("A:{0:.3f}  B:{1:.3f}".format(angle1,angle2), 2, 0)
        print('angle 1 ', angle1)
        print('angle 2 ', angle2)
        print('angle 3 ', angle3)
        
        if angle1 < 0.01 and angle1 > -0.01 and angle2 < 0.01 and angle2 > -0.01:
            print('Leveling Done')
            print('-- Final Sensor Values --')
            print('Sensor 1: {}  Sensor 2: {}'.format(angle1, angle2))
            
            mylcd.lcd_display_string_pos("Leveling Done".format(angle1,angle2), 1, 1)
            break

#         perInch1 = (angle1 - oldAngle1) / newH[0]
#         perInch2 = (angle2 - oldAngle2) / newH[1]
#         perInch3 = (angle3 - oldAngle3) / newH[2]

        oldAngle1 = angle1
        oldAngle2 = angle2
        oldAngle3 = angle3

        oldPos = newPos
        
