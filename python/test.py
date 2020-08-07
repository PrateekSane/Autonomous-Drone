import time
import pigpio
import RPi.GPIO as GPIO
from gpiozero import Button
import mpudata as mpu
import numpy as np
import PID_Loops as pid
pi = pigpio.pi()


TIMEOUT = 5
m1 = 13 #front right CW
m2 = 5 #front left CCW
m3 = 6 #back right CCW
m4 = 12  #back left CW
motors = [m1, m2, m3, m4]


#translateGyroData (dataRep, dataFreq, axis, resultX, resultY, resultZ)
#def gyroDataMovingAverage(self, rollLength, dataRep, dataFreq, axisNeeded)
#loops = pid.control_loops()
#datasetBE, measurementCount = loops.gyroDataBoundedExclusion (10, .05, 'z')
loops = pid.control_loops(motors)
#netset, valset, measurementCount = loops.gyroDataMovingAverage(5, .02, 'z', 9)
#dataset = [netset, valset]
#mpu.measurementVsCount(measurementCount, dataset)
#    def holdZ (self, kp, ki, kd, curspeed, integralThreshold, runTime, motors):

speedSet, errorSet, tickerSet  = loops.holdZ( .1, 2, .2, 1500, 250, 10, motors)
#dataset = [errorSet, speedSet]
#mpu.measurementVsCount(tickerSet, dataset)
for motor in motors:
    pi.set_servo_pulsewidth(motor, 1000)
