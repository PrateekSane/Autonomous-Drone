import time
import pigpio
import RPi.GPIO as gpio
import mpudata as mpu
import PID_Loops as PID
pi = pigpio.pi()

#avg: 0.0006363727101688666 -0.02404023398668752 -0.5001133701156375
#stdev: 0.0046486083758466605 0.0009675719684475337 0.08433379574532719

#these are threshold for change
#avg vals for g[x, y] for 5pct error
#xThreshold = .031 ,,,, yThreshold = .495

#for Z, initial value is based on starting position
#Therefore the inital value needs to be calculated and then use StdevZ for the error

#About 1.8 stdev over to include most


#stdev 10 sec .05 = .0033
#stdev 10 sec .1 = .004

m1 = 13 #front right
m2 = 5 #front left
m3 = 6 #back right
m4 = 12  #back left

motors = [m1, m2, m3, m4]

zkp, zkd, zki = 1, 1, 1
xykp, xykd, xyki = 1,1,1

curSpeed = 127
freq = .3

