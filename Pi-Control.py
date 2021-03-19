import serial
import time
import numpy as np

sensor1 = ''
sensor2 = ''

platform1 = 'COM3'
platform2 = ''
platform3 = ''


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
    ser.write(bytes(int(steps)))


def findThirdAngle(ang1, ang2, distBetween):  # gives the third angle relative to the 1st platform
    pos1 = distBetween * np.sin(ang1 * np.pi / 180)
    pos2 = distBetween * np.sin(ang2 * np.pi / 180)
    d = pos2 - pos1
    ang3 = np.arctan((d / distBetween) * np.pi * 180)
    return ang3


if __name__ == "__main__":
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

    # Calibration - Sensors
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
        time.sleep(.1)

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
        time.sleep(0.1)
    print('calibrated')

    calibAngle1 = getSensorData(sen1)
    calibAngle2 = getSensorData(sen2)

    # calib is moving the middle actuator up 1 inch
    perInch1 = calibAngle1 - zeroAngle1  # angle from plat2 to plat1
    perInch2 = calibAngle2 - zeroAngle2  # angle from plat2 to plat3
    perInch3 = (perInch1 + perInch2) / 2  # angle from plat1 to plat3

    distBetween = 1 / np.tan(perInch3 * np.pi / 180)

    homed[1] = False
    homeActuator(plat2)

    while not homed[1]:
        line = plat2.readline().decode('utf-8').rstrip()
        if line == 'done':
            homed[1] = True
        time.sleep(0.1)
    print('homed')

    # Initialize important values
    MIN_ACTUATOR = 0
    MAX_ACTUATOR = 7.5

    oldAngle1 = getSensorData(sen1)
    oldAngle2 = getSensorData(sen2)
    oldAngle3 = findThirdAngle(oldAngle1, oldAngle2, distBetween)
    oldPos = np.array([0, 0, 0])
    retry = 0

    while True:
        moving = [True, True, True]
        # this uses linear algebra to find the amount to move each actuator quickly
        a = np.array([[perInch1, perInch1, 0],
                  [0, perInch2, perInch2],
                  [perInch3, 0, perInch3]])
        b = np.array([oldAngle1, oldAngle2, oldAngle3])

        newH = np.linalg.solve(a, b)  # array of how far each platform needs to move
        newPos = newH + oldPos

        if np.amin(newPos) < MIN_ACTUATOR:
            newPos = newPos + np.amin(newPos)
            newH = newH + np.amin(newPos)  # corrects for if the result tells platform to move lower than it can
            if np.amax(newPos) > MAX_ACTUATOR:
                retry += 1
                if retry < 5:
                    print('Could not solve for angle... retrying...')
                    continue
                else:
                    print('Angle of ground is too much for platform to handle')
                    break
            retry = 0

        if np.amin(newPos) > MAX_ACTUATOR:
            newPos = newPos - (np.amax(newPos) - MAX_ACTUATOR)
            newH = newH - (np.amax(newPos) - MAX_ACTUATOR)
            if np.amax(newPos) < MIN_ACTUATOR:
                retry += 1
                if retry < 5:
                    print('Could not solve for angle... retrying...')
                    continue
                else:
                    print('Angle of ground is too much for platform to handle')
                    break
            retry = 0

        moveActuator(plat1, newH[0])
        moveActuator(plat2, newH[1])
        moveActuator(plat3, newH[2])

        while all(moving):
            time.sleep(0.5)
            for i in range(len(plats)):
                line = plats[i].readline().decode('utf-8').rstrip()
                if line == 'done':
                    moving[i] = False

        angle1 = getSensorData(sen1)
        angle2 = getSensorData(sen2)
        angle3 = findThirdAngle(angle1, angle2, distBetween)

        perInch1 = (angle1 - oldAngle1) / newH[0]
        perInch2 = (angle2 - oldAngle2) / newH[1]
        perInch3 = (angle3 - oldAngle3) / newH[2]

        oldAngle1 = angle1
        oldAngle2 = angle2
        oldAngle3 = angle3

        oldPos = newPos

