import serial
import time


plat1 = serial.Serial('COM3', 9600, timeout=1)
print(plat1.isOpen())
print(plat1)

time.sleep(2)

plat1.write(b'bhome')

time.sleep(1)

print(plat1.readline())
