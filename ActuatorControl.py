import serial


class ActuatorControl:

    def __init__(self, actuator_path):
        self.ser = serial.Serial(actuator_path, 11520, timeout=1)
        self.pos = 0
        self.moving = False

    def move(self, inches):
        steps = int(inches * 11792)
        self.ser.write(('n' + str(int(steps))).encode('ascii'))
        self.moving = True
        self.pos += inches

    def calibrate(self):
        self.ser.write(b"bc")
        self.moving = True

    def home(self):
        self.ser.write(b"bh")
        self.moving = True

    def isMoving(self):
        while self.ser.in_waiting:
            line = self.ser.readline().decode('utf-8').rstrip()
            if line == "d":
                self.moving = False
        return self.moving







