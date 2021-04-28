import RPi.GPIO as GPIO
import RPi_I2C_driver
from ActuatorControl import ActuatorControl
from SensorControl import SensorControl
import time
import numpy as np


class PlatformSystem:

    def __init__(self, serial_paths, final_tolerance=0.008):
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
        sensorL_low = self.left_sensor.getAngle()
        sensorR_low = self.left_sensor.getAngle()
        self.center_actuator.calibrate()
        while self.center_actuator.isMoving():
            time.sleep(0.005)
        sensorL_high = self.left_sensor.getAngle()
        sensorR_high = self.left_sensor.getAngle()
        self.perInchL = sensorL_high - sensorL_low
        self.perInchR = sensorR_high - sensorR_low
        self.homeActuators()
        
    def homeActuators(self):
        self.left_actuator.home()
        self.center_actuator.home()
        self.right_actuator.home()
        
        while self.left_actuator.isMoving() or self.center_actuator.isMoving() or self.right_actuator.isMoving():
            time.sleep(0.05)
        
    def move(self):
        self.update_sensors()
        move_arr = self.calculate_movement()
        move_arr, pos = self.check_position(move_arr)
        if move_arr != "err":
            self.move_actuators(move_arr)
            return pos
        else:
            return -1
        
    
    def update_sensors(self):
        self.left_sensor.getAngle()
        self.right_sensor.getAngle()
        
    def calculate_movement(self):
        move_arr = [0, 0, 0]
        move_arr[0] = self.left_sensor.angle / self.perInchL
        move_arr[2] = self.right_sensor.angle / self.perInchR
        return move_arr
    
    def check_position(self, move_arr):
        MAX_ACTUATOR = 8
        MIN_ACTUATOR = 0
        for i in range(3):
            if move_arr[i] > 0.5:
                move_arr[i] = 0.5
            if move_arr[i] < -0.5:
                move_arr[i] = -0.5
                
        pos = [self.left_actuator.pos, self.center_actuator.pos, self.right_actuator.pos]
        newPos = np.add(move_arr, pos)
        
        if np.amin(newPos) < MIN_ACTUATOR:
            correct_amt = np.abs(np.amin(newPos))
            newPos = np.add(newPos, correct_amt)
            move_arr = np.add(move_arr, correct_amt)  # corrects for if the result tells platform to move lower than it can
            if np.amax(newPos) > MAX_ACTUATOR:
                return 'err'
        if np.amax(newPos) > MAX_ACTUATOR:
            correct_amt = (np.amax(newPos) - MAX_ACTUATOR)
            newPos = np.subtract(newPos, correct_amt)
            move_arr = np.subtract(move_arr, correct_amt)
            if np.amax(newPos) < MIN_ACTUATOR:
                return 'err'
        return move_arr, newPos
        
    def move_actuators(self, move_arr):
        self.left_actuator.move(move_arr[0])
        self.center_actuator.move(move_arr[1])
        self.right_actuator.move(move_arr[2])
        
    def move_done(self):
        while self.left_actuator.isMoving() or self.center_actuator.isMoving() or self.right_actuator.isMoving():
            return False
        return True

    def isLevel(self): 
        angleL = self.left_sensor.angle
        angleR = self.right_sensor.angle
        if self.tol > angleL > -self.tol and self.tol > angleR > -self.tol:
            return True
        else:
            return False

    def get_angles(self):
        return self.left_sensor.angle, self.right_sensor.angle

        
    
    
