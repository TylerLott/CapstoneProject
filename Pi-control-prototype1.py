import serial
import time
import numpy as np

# ### This code only operates a single platform and sensor -- for use in first prototype video

sensor1 = '/dev/ttyUSB2'

platform1 = '/dev/ttyUSB1'


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
    ser.write("bh".encode('ascii'))


# Extends the actuator an inch
def calibActuator(ser):
    ser.write("bc".encode('ascii'))


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
    # Initialize all serial connections
    sen1 = serial.Serial(sensor1, 9600, timeout=10)
    sen1.flush()

    plat1 = serial.Serial(platform1, 14400, timeout=10)
    plat1.flush()

    time.sleep(4)
    print('connected')
    # Calibration - Sensors
    sen1.write(b"do calibr\n")

    cal1 = False

    while not cal1:
        if sen1.readline().decode('utf-8').rstrip() == 'done':
            cal1 = True
        time.sleep(.1)

    # Calibration - Actuators
    print('sensor done')
    homed = False
    print('homing...')
    homeActuator(plat1)
    plat1.flushInput()

    while not homed:
        time.sleep(0.5)
        line = plat1.readline().decode('utf-8').rstrip()
        if line == 'done':
            homed = True
    print('homed')

    zeroAngle1 = getSensorData(sen1)
    print('calibrating...')
    calibrated = False
    calibActuator(plat1)
    line = ''
    plat1.flushInput()

    while not calibrated:
        line = plat1.readline().decode('utf-8').rstrip()
        print(line)
        if line == 'done':
            calibrated = True
        time.sleep(0.1)
    print('calibrated')

    calibAngle1 = getSensorData(sen1)

    # calib is moving the middle actuator up 1 inch
    perInch1 = calibAngle1 - zeroAngle1  # angle from plat2 to plat1

    distBetween = 1 / np.tan(perInch1 * np.pi / 180)

    homed = False
    print('homing...')
    homeActuator(plat1)

    while not homed:
        time.sleep(0.5)
        line = plat1.readline().decode('utf-8').rstrip()
        if line == 'done':
            homed = True
    print('homed')

    # Initialize important values
    MIN_ACTUATOR = 0
    MAX_ACTUATOR = 7.5

    oldAngle1 = getSensorData(sen1)
    oldPos = 0
    retry = 0

    while True:
        moving = True

        h = oldAngle1 / perInch1
        if h > 2:
            h = 2
        if h < -2:
            h = -2
        newPos = oldPos + h
        
        print('angle ', oldAngle1)
#         print('moving ', h)

        moveActuator(plat1, h)
        line=''
        while moving:
            time.sleep(0.5)
            line = plat1.readline().decode('utf-8').rstrip()
            if line == 'done':
                moving = False

        angle1 = getSensorData(sen1)

        if angle1 < 0.005 and angle1 > -0.005:
            break

#         perInch1 = (angle1 - oldAngle1) / h

        oldAngle1 = angle1
        oldPos = newPos

    print("Leveled")
