import RPi.GPIO as GPIO
import time

import RPi_I2C_driver
from DBWriter import DBWriter
from PlatformSystem import PlatformSystem

# SERIAL CONNECTIONS TO ARDUINOS
# These should stay the same if the plugs are all in the same spot.
# Currently:
# blue - sensor - top USB3.0 port
# yellow - sensor - bottom USB2.0 port
# USB dongle - bottom USB3.0 port
# on dongle, from cord side, yellow -> blue -> red
sensorL = '/dev/ttyUSB0'
sensorR = '/dev/ttyUSB1'

platformL = '/dev/ttyUSB4'
platformC = '/dev/ttyUSB3'
platformR = '/dev/ttyUSB2'

serial_paths = [sensorL, sensorR, platformL, platformC, platformR]

# GPIO PIN FOR PHYSICAL INTERFACE
btnPress = False

def button_callback(channel):
    global btnPress
    btnPress = True

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(16, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(16,GPIO.RISING,callback=button_callback)

# I2C LCD DISPLAY
mylcd = RPi_I2C_driver.lcd()

mylcd.lcd_display_string_pos("Platform Booting", 1, 0)
mylcd.lcd_display_string_pos("....", 2, 6)

# DATABASE WRITER
db_path = '/home/pi/Desktop/CapstoneProject/database/platform.db'
db_data = [0,0,0,0,0]
db_writer = DBWriter(db_path)

# INITIALIZE PLATFORM
try:
    plat_sys = PlatformSystem(serial_paths)
    time.sleep(5)
    db_writer.writeTerm('[SETUP]  serial connections made')
    print('serial connection made')
except Exception as e:
    err = '[ERROR]  There was an error initializing'
    db_writer.writeTerm(err)
    db_writer.writeTerm(str(e))
    print(e)
    
    

# LOWER PLATFORMS
mylcd.lcd_clear()
mylcd.lcd_display_string_pos("Press to Lower", 1, 1)

while not btnPress:
    time.sleep(.05)
btnPress = False

mylcd.lcd_clear()
mylcd.lcd_display_string_pos("Lowering...", 1, 3)
plat_sys.homeActuators()
db_writer.writeTerm('[SETUP]  lowering platforms')

plat_sys.update_sensors()
db_data[1], db_data[3] = plat_sys.get_angles()
db_writer.writeData(db_data)
db_writer.writeTerm('[SETUP]  lowering complete')
db_writer.writeTerm(f'[SETUP]  initial sensor readings - L:{db_data[1]}  R:{db_data[3]}')

# Calibrate Sensors
# the arduino's 5V is too weak to power the servos so this is omitted
# plat_sys.calibrateSensors()
  
  
# Calibrate Actuators
mylcd.lcd_clear()
mylcd.lcd_display_string_pos("Press to", 1, 4)
mylcd.lcd_display_string_pos("Calibrate", 2, 4)

while not btnPress:
    time.sleep(.05)
btnPress = False

mylcd.lcd_clear()
mylcd.lcd_display_string_pos("Calibrating...", 1, 1)

plat_sys.calibrateActuators()
db_writer.writeTerm('[SETUP]  calibration complete')

# Start Leveling
mylcd.lcd_clear()
mylcd.lcd_display_string_pos("Press to", 1, 4)
mylcd.lcd_display_string_pos("Start", 2, 5)

while not btnPress:
    time.sleep(.05)
btnPress = False

mylcd.lcd_clear()
mylcd.lcd_display_string_pos("Leveling...", 1, 3)
mylcd.lcd_display_string_pos("L:{0:.2f}  R:{1:.2f}".format(db_data[1],db_data[3]), 2, 0)

db_writer.writeTerm(' ')
db_writer.writeTerm('[MOVING] leveling started')

while not plat_sys.isLevel():
    
    # pause condition during leveling
    if btnPress:
        db_writer.writeTerm('[STOPPED] leveling paused by user')
        mylcd.lcd_clear()
        mylcd.lcd_display_string_pos("Leveling Paused", 1, 0)
        mylcd.lcd_display_string_pos("L:{0:.2f}  R:{1:.2f}".format(db_data[1],db_data[3]), 2, 0)
        btnPress = False
        
        while not btnPress:
            time.sleep(.05)
        btnPress = False

    # see if user has input anything from debug
    type, amount = db_writer.readUserIn()
    if type != 'none':
	if type == 'actL':
	    plat_sys.left_actuator.move(amount)
	if type == 'actC':
	    plat_sys.center_actuator.move(amount)
	if type == 'actR':
	    plat_sys.right_actuator.move(amount)
    
    pos = plat_sys.move()
    db_data[0], db_data[2], db_data[4] = pos
    db_writer.writeTerm('[MOVING] moving to new position - L:{1:0.3f} C:{1:0.3f} R:{1:0.3f}'.format(db_data[0], db_data[2], db_data[4]))
    while not plat_sys.move_done():
        plat_sys.update_sensors()
        db_data[1], db_data[3] = plat_sys.get_angles()
        mylcd.lcd_display_string_pos("L:{0:.2f}  R:{1:.2f}".format(db_data[1],db_data[3]), 2, 0)
        db_writer.writeTerm(f'[SENSOR] sensor angles - L:{db_data[1]}  R:{db_data[3]}') 
        db_writer.writeData(db_data)
    
mylcd.lcd_clear()
mylcd.lcd_display_string_pos("Platform Level", 1, 1)
mylcd.lcd_display_string_pos("L:{0:.2f}  R:{1:.2f}".format(db_data[1],db_data[3]), 2, 0)

db_writer.writeTerm(' ')
db_writer.writeTerm(f'         ----PLATFORM LEVELED----') 
db_writer.writeTerm(f'[DONE]   final sensor angles - L:{db_data[1]}  R:{db_data[3]}')
db_writer.writeTerm(f'[DONE]   final positions - L:{1:0.3f} C:{1:0.3f} R:{1:0.3f}'.format(db_data[0], db_data[2], db_data[4]))

print(f'final sensor angles: L:{db_data[1]} R:{db_data[3]}')




