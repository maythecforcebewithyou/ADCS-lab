#sensor_calc.py
import time
import numpy as np
#import adafruit_fxos8700
#import adafruit_fxas21002c
import adafruit_bno055
import time
import os
import board
import busio

i2c = busio.I2C(board.SCL, board.SDA)
#sensor1 = adafruit_fxos8700.FXOS8700(i2c)
#sensor2 = adafruit_fxas21002c.FXAS21002C(i2c)
sensor = adafruit_bno055.BNO055_I2C(i2c)


#Activity 1: RPY based on accelerometer and magnetometer
def roll_am(accelX,accelY,accelZ):
    roll = np.arctan(accelY/(accelX**2 + accelZ**2)**0.5) # from doc
    return roll

def pitch_am(accelX,accelY,accelZ):
    pitch = np.arctan(accelX/(accelY**2 + accelZ**2)**0.5) # from doc
    return pitch

def yaw_am(accelX,accelY,accelZ,magX,magY,magZ):
    roll = roll_am(accelX,accelY,accelZ)
    pitch = pitch_am(accelX,accelY,accelZ)
    mag_x = magX*np.cos(pitch) + magY*np.sin(roll)*np.sin(pitch) + magZ*np.cos(roll)*np.sin(pitch)
    mag_y = magY*np.cos(roll) - magZ*np.sin(roll)
    return (180/np.pi)*np.arctan2(-mag_y, mag_x)

#Activity 2: RPY based on gyroscope
def roll_gy(prev_angle, delT, gyro):
    roll = prev_angle+delT*gyro
    return roll
def pitch_gy(prev_angle, delT, gyro):
    pitch = prev_angle+delT*gyro
    return pitch
def yaw_gy(prev_angle, delT, gyro):
    yaw = prev_angle+delT*gyro
    return yaw

def set_initial(mag_offset = [0,0,0]):
    #Sets the initial position for plotting and gyro calculations.
    print("Preparing to set initial angle. Please hold the IMU still.")
    time.sleep(3)
    print("Setting angle...")
    accelX, accelY, accelZ = sensor.acceleration #m/s^2
    magX, magY, magZ = sensor.magnetic
    #Calibrate magnetometer readings. Defaults to zero until you
    
    offset = calibrate_mag()
    #offset = calibrate_gyro()
    
    #write the code
    magX = magX - offset[0]
    magY = magY - offset[1]
    magZ = magZ - offset[2]
    roll = roll_am(accelX, accelY,accelZ)
    pitch = pitch_am(accelX,accelY,accelZ)
    yaw = yaw_am(accelX,accelY,accelZ,magX,magY,magZ)
    print("Initial angle set.")
    return [roll,pitch,yaw]

def calibrate_mag():
    #TODO: Set up lists, time, etc
    minX, minY, minZ = sensor.magnetic
    maxX, maxY, maxZ = sensor.magnetic
    timer = time.time()
    print("Preparing to calibrate magnetometer. Please wave around.")
    #time.sleep(3)
    while(time.time() < timer + 3):
        magX, magY, magZ = sensor.magnetic
        if magX < minX: minX = magX
        elif magX > maxX: maxX = magX
        if magY < minY: minY = magY
        elif magY > maxY: maxY = magY
        if magZ < minZ: minZ = magZ
        elif magZ > maxZ: maxZ = magZ
        #print(time.time() - timer)
    print("Calibrating...")
    #TODO: Calculate calibration constants
    avgX = (minX + maxX) / 2
    avgY = (minY + maxY) / 2
    avgZ = (minZ + maxZ) / 2
    #print("Calibration complete.")
    return [avgX, avgY, avgZ]

def calibrate_gyro():
    #TODO
    #print("Preparing to calibrate gyroscope. Put down the board and do not touch it.")
    #time.sleep(3)
    #print("Calibrating...")
    #TODO
    #print("Calibration complete.")
    return [0, 0, 0]
