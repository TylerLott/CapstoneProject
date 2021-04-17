import RPi.GPIO as GPIO
import RPi_I2C_driver
from ActuatorControl import ActuatorControl
from SensorControl import SensorControl
import time
import numpy as np


class PlatformSystem:

    def __int__(self, serial_paths, final_tolerance=0.008):
        # serial_paths should be ['/dev/ttyUSB0', '/dev/ttyUSB2', '/dev/ttyUSB1', '/dev/ttyUSB2', '/dev/ttyUSB3']
        # For the raspberry pi being used

        # SENSORS
        self.left_sensor = SensorControl(serial_paths[0])
        self.right_sensor = SensorControl(serial_paths[1])
        # ACTUATORS
        self.left_actuator = ActuatorControl(serial_paths[2])
        self.center_actuator = ActuatorControl(serial_paths[3])
        self.right_actuator = ActuatorControl(serial_paths[4])
        
        self.perInchL = 0
        self.perInchR = 0
        self.tol = final_tolerance

    def calibrateSensors(self):
        l = self.left_sensor.calibrate()
        r = self.right_sensor.calibrate()

    def calibrateActuators(self):
        self.homeActuators()
        sensorL_low = self.left_sensor.getAngle()
        sensorR_low = self.left_sensor.getAngle()
        self.center_actuator.calibrate()
        while self.center_actuator.isMoving():
            time.sleep(0.005)
        sensorL_high = self.left_sensor.getAngle()
        sensorR_high = self.left_sensor.getAngle()
        self.perInchL = sensorL_high - sensorL_low
        self.perInchR = sensorR_high - sensorR_low
        
    def homeActuators(self):
        self.left_actuator.home()
        self.center_actuator.home()
        self.right_actuator.home()
        
        while not self.left_actuator.isMoving and not self.center_actuator.isMoving() and not self.right_actuator.isMoving():
            return 1
        
    def move(self):
        self.update_sensors()
        move_arr = self.calculate_movement()
        move_arr = self.check_position(move_arr)
        if move_arr != "err":
            self.move_actuators(move_arr)
            return 1
        else:
            return -1
        
    
    def update_sensors(self):
        self.left_sensor.getAngle()
        self.left_sensor.getAngle()
        
    def calculate_movement(self):
        move_arr = [0, 0, 0]
        move_arr[0] = self.left_sensor.angle / self.perInchL
        move_arr[2] = self.left_sensor.angle / self.perInchR
        return move_arr
    
    def check_position(self, move_arr):
        MAX_ACTUATOR = 8
        MIN_ACTUATOR = 0
        for i in range(3):
            if move_arr[i] > 2:
                move_arr[i] = 2
            if move_arr[i] < -2:
                move_arr[i] = -2
                
        pos = [self.left_actuator, self.center_actuator, self.right_actuator]
        newPos = move_arr + pos
        
        if np.amin(newPos) < MIN_ACTUATOR:
            correct_amt = np.abs(np.amin(newPos))
            print('amin: ', correct_amt)
            newPos = newPos + correct_amt
            move_arr += correct_amt  # corrects for if the result tells platform to move lower than it can
            if np.amax(newPos) > MAX_ACTUATOR:
                return 'err'
        if np.amax(newPos) > MAX_ACTUATOR:
            correct_amt = (np.amax(newPos) - MAX_ACTUATOR)
            newPos = newPos - correct_amt
            move_arr -= correct_amt
            if np.amax(newPos) < MIN_ACTUATOR:
                return 'err'
        return move_arr
        
    def move_actuators(self, move_arr):
        self.left_actuator.move(move_arr[0])
        self.center_actuator.move(move_arr[1])
        self.right_actuator.move(move_arr[2])
        while self.left_actuator.isMoving():
            time.sleep(0.005)
        while self.center_actuator.isMoving():
            time.sleep(0.005)
        while self.right_actuator.isMoving():
            time.sleep(0.005)

    def check_level(self): # return 1 if level 0 if not
        angleL = self.left_sensor.angle
        angleR = self.right_sensor.angle
        if self.tol > angleL > -self.tol and self.tol > angleR > -self.tol:
            return 1
        else:
            return 0

    def get_angles(self):
        return self.left_sensor.angle, self.right_sensor.angle

        
    
    
