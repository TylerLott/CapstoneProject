import serial


class SensorControl:

    def __init__(self, sensor_path):
        self.ser = serial.Serial(sensor_path, 11520, timeout=1)
        self.angle = 180

    def getAngle(self):
        self.ser.write(b"g\n")
        while self.ser.in_waiting:
            line = self.ser.readline().decode('utf-8').rstrip()
            if line == "r":
                self.getAngle()
            if line == "e":
                return 999 # returns 999 when sensor error
            self.angle = float(line)
            return self.angle

    def calibrate(self):
        self.ser.write(b"c\n")
        while self.ser.in_waiting:
            line = self.ser.readline().decode('utf-8').rstrip()
            if line == "r":
                self.getAngle()
            if line == "e":
                return 999  # returns 999 when sensor error
            if line == "d":
                return 1 # returns 1 if finished




