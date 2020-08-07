

    def xyPID (motors, curSpeed, integralThreshold, kp, ki, kd, runTime, freq): 
        gyroData = mpu.get_gyro_data
        xErr = gyroData[x]
        yErr = gyroData[y]
        totSum = 0
        
        while i < runTime:
            gyroData = mpu.get_gyro_data
            #accelData = mpu.get_accel_data
            
            xErr = gyroData[x]
            yErr = gyroData[y]
            
            if xErr == 0:
                xErr = .01
            
            combinedErr = float(y/x)
            #have to convert to rad
            
            angleErr = np.cos(combinedErr)
            
            
            deriv = err - prevErr
            
            val = kp*err + kd*derv
            
            if totSum > integralThreshold:
                val += ki * totSum
                
            
            for motor in side[0]:
                gpioPWM(motor, curSpeed + val)
            
            prevErr = err
            
            totSum += err
            val = 0
            i += freq
            sleep(freq)
        
####################################################################################
#button on 17
#LED on 
button = Button(17)
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(21, GPIO.OUT) #yellow
GPIO.setup(27, GPIO.OUT) #RED

buttonPress = False
i = 0
while(i < TIMEOUT):
    if button.is_pressed:
        buttonPress = not buttonPress
        time.sleep(.3)
        i += 1
    if buttonPress:
        GPIO.output(21, GPIO.HIGH)
        GPIO.output(27, GPIO.LOW)
    else:
        GPIO.output(27, GPIO.HIGH)
        GPIO.output(21, GPIO.LOW)
    
    
GPIO.output(21, GPIO.LOW)
GPIO.output(27, GPIO.LOW)
#############################################################################################
speed = 1000
pi.set_servo_pulsewidth(m3, speed)
time.sleep(.055)
for motor in motors:
     pi.set_servo_pulsewidth(motor, speed)

time.sleep(1)
pi.set_servo_pulsewidth(m3, 1000)
for motor in motors:
    pi.set_servo_pulsewidth(motor, 1000)
time.sleep(2)
