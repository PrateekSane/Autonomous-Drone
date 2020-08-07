import time
import pigpio
import RPi.GPIO as gpio
import mpudata as mpu
import numpy as np

pi = pigpio.pi()
#m1 = 13 #front right CW
#m2 = 5 #front left CCW
#m3 = 6 #back right CCW
#m4 = 12  #back left CW

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

class control_loops:
    def __init__(self, motors):
        
        self.means = {'x': 0.004648608, 'y': -0.02404023, 'z': -0.50011337}
        self.stds =  {'x': 0.000636372, 'y': 0.000967571, 'z': 0.08433379 }
        self.results = {'x': self.means['x'], 'y': self.means['y'], 'z': self.means['z']}
        self.axises = ['x', 'y', 'z']
        self.zscore = 2.5
        self.m1, self.m2, self.m3, self.m4 = motors[0], motors[1], motors[2], motors[3]
        self.motorList = [self.m1, self.m2, self.m3, self.m4]
        self.motorSpeeds = {self.m1: 1000, self.m2: 1000, self.m3: 1000, self.m4: 1000}
        self.mpu = mpu.mpu6050(0x68)
    
    def gyroDataMovingAverage(self, dataRep, dataFreq, curNetval, axis, rollLength):
        elapsedTime = 0
        #resultset = []
        #netset = []
        #measurementCount = []
        mescount = 0
        if curNetval:
            netval = curNetval
        else:
            netval = self.means[axis]
            
        curvals = []
        
        upper = self.means[axis] + self.zscore*self.stds[axis]
        lower = self.means[axis] - self.zscore*self.stds[axis]
        while(elapsedTime < dataRep):
            gyroData = self.mpu.get_gyro_data()
            gyroVal = gyroData[axis]
            mescount+=1
            
            if mescount < rollLength:
                curvals.append(gyroVal)
                self.results[axis] == np.mean(curvals)
                        
            else:
                curvals.append(gyroVal)
                self.results[axis] = np.mean(curvals[mescount - rollLength: mescount])
            
            current = self.results[axis]
            if current > upper or current < lower:
                netval += self.results[axis]
            
            #netset.append(netval)
            #resultset.append(self.results[axis])
            #measurementCount.append(mescount)
            #print(str(self.results[axis]) + '               ' + str(gyroData[axis]) + '               ' + str(netset[mescount-1]))
            elapsedTime += dataFreq            
            time.sleep(dataFreq)
        #return netset, resultset, measurementCount
        return netval
    
    
#################################################################################################################################################################################
    def holdZ (self, kp, ki, kd, curspeed, integralThreshold, runTime, motors):
        dataFreq = .0133
        dataRep = .1999
        axis = 'z'
        zGyro = self.gyroDataMovingAverage(dataRep, dataFreq, None, axis, 10)
        holdZRuntime = 0
        err = zGyro - self.means[axis]
        prevErr = err
        totSum = 0
        upper = self.means[axis] + self.zscore*self.stds[axis]
        lower = self.means[axis] - self.zscore*self.stds[axis] 
        icont = 0
        #plotting info
        ticker = 0
        tickerSet = []
        errorSet = []
        speedSet = []
        while holdZRuntime < runTime:
            zGyro = self.gyroDataMovingAverage(dataRep, dataFreq, None, axis, 10)
            start = time.perf_counter() 
            if zGyro <= upper and zGyro >= lower:
                print('target achieved')
                time.sleep(.2)
            
            err = zGyro - self.means[axis]
            
            deriv = err - prevErr
            
            pcont = kp*err
            dcont = kd*deriv
            
            if abs(totSum) > integralThreshold:
                icont = ki * totSum
            val = pcont + icont + dcont
            cont = {'p': pcont, 'i': icont, 'd': dcont}
            speed = curspeed + val
            if speed >= 2000:
                speed= 2000
            if speed <= 1000:
                speed= 1000
            curspeed = speed
            for motor in motors:
                pi.set_servo_pulsewidth(motor, speed)
            
 
            
            #  
            prevErr = err
            totSum += err
            val = 0
            holdZRuntime += dataRep
            ticker += 1
            #print(f'error: {err},\t speed: {speed},\t totSum: {totSum} \t ')
            tickerSet.append(ticker)
            speedSet.append(speed)
            errorSet.append(err)
            finish = time.perf_counter()
            print(round(finish - start, 4))
            
        for motor in motors:
                pi.set_servo_pulsewidth(motor, 0)        
        return speedSet, errorSet, tickerSet        
            
            
#################################################################################################################################################################################
    def holdXY (self, x_constants, y_constants, curspeed, ITs, runTime):
        dataFreq = .0133
        dataRep = .1999
        x = 'x'
        y= 'y'
        xGyro = self.gyroDataMovingAverage(dataRep, dataFreq, None, x, 10)
        yGyro = self.gyroDataMovingAverage(dataRep, dataFreq, None, y, 10)
        holdXYRuntime = 0
        
        X_err = xGyro - self.means[x]
        Y_err = yGyro - self.means[y]
        x2 = X_err ** 2
        y2 = Y_err ** 2
        xy = x2 + y2
        #X_combErr = x2 / xy
        #Y_combErr = y2 / xy
        X_prevErr = X_err
        Y_prevErr = Y_err
        X_totSum = 0
        Y_totSum = 0
        xkp, xki, xkd = x_constants[0], x_constants[1], x_constants[2]
        ykp, yki, ykd = y_constants[0],y_constants[1], y_constants[2]
        xIT, yIT = ITs[0], ITs[1]
        
        meansXY = self.means[x] ** 2 + self.means[y] ** 2
        stdevsXY = self.stds[x] ** 2 + self.stds[y] ** 2
        margin = meansXY + self.zscore * stdevsXY
        #plotting info
        ticker = 0
        tickerSet = []
        errorSet = []
        speedSet = []
        while holdZRuntime < runTime:
            xGyro = self.gyroDataMovingAverage(dataRep, dataFreq, None, x, 10)
            yGyro = self.gyroDataMovingAverage(dataRep, dataFreq, None, y, 10)
            
            X_err = xGyro - self.means[x]
            Y_err = yGyro - self.means[y]
            
            if xy < margin : 
                print('target achieved')
                continue
            
            X_prevErr = X_err
            Y_prevErr = Y_err
            
            Xderiv = err - prevErr
            Yderiv = err - prevErr
            
            Xval = xkp*err
            Yval = ykp*err
            Xval += xkd*deriv
            Yval += ykd*deriv
            
            if abs(X_totSum) > xIT:
                Xval -= xki * X_totSum
                #CHECK SIGN
            if abs(Y_totSum) > yIT:
                Yval -= yki * Y_totSum
                #CHECK SIGN
            speed = curspeed + val
            curspeed = speed
            for motor in motorList:
                
                pi.set_servo_pulsewidth(motor, speed)
            
 
            
            #  
            prevErr = err
            totSum += err
            val = 0
            holdZRuntime += dataRep
            ticker += 1
            print('error: ',str(err), '\t', 'speed: ', str(speed), '\t','totSum: ', str(totSum))
            tickerSet.append(ticker)
            speedSet.append(speed)
            errorSet.append(err)
            
        for motor in motors:
                pi.set_servo_pulsewidth(motor, 0)        
        return speedSet, errorSet, tickerSet        
#################################################################################################################################################################################
 
    def moveZ (self, target, kp, ki, kd, curspeed, integralThreshold, runTime, motors): 
        
        dataFreq = .0125
        dataRep = .16
        zGyro = self.gyroDataMovingAverage(dataRep, dataFreq, self.means['z'], 'z', 5)
        i = 0
        err = zGyro - target
        prevErr = err
        totSum = 0
        upper = target + self.zscore*self.stds['z']
        lower = target - self.zscore*self.stds['z']
        #print(zGyro, upper, lower)
        while zGyro >= upper or zGyro <= lower:
            zGyro = self.gyroDataMovingAverage(dataRep, dataFreq, zGyro, 'z', 5)
            
            if i >= runTime: 
                break
               
            if zGyro <= upper and zGyro >= lower:
                print('target achieved')
                break
            
            err = zGyro - target
            deriv = err - prevErr
            
            val = kp*err
            #val += kd*derv
            
            #if totSum > integralThreshold:
                #val += ki * totSum
                
            speed = curspeed + val
            curspeed = speed
            #for motor in motors:
            #    pi.set_servo_pulsewidth(motor, speed)
            
            print('error: ',str(err), '\t', 'gyroVal: ', str(zGyro) )
            
            # 'speed: ', str(speed), '\t',
            prevErr = err
            totSum += err
            val = 0
            i += dataRep
            
            
#################################################################################################################################################################################
    
    def gyroDataBoundedExclusion (self, dataRep, dataFreq, axis):
        i = 0
        dataset = []
        measurementCount = []
        mescount = 0
        while(i < dataRep):
            gyroData = self.mpu.get_gyro_data()
            gyroVal = gyroData[axis]
            upper = self.means[axis] + self.zscore*self.stds[axis]
            lower = self.means[axis] - self.zscore*self.stds[axis]
            
            if gyroVal >= upper or gyroVal <= lower:
                self.results[axis] += gyroVal
            
            dataset.append(self.results[axis])
            mescount+=1
            measurementCount.append(mescount)
            i += dataFreq
            print(str(self.results[axis]) + '               ' + str(gyroData[axis]))
            time.sleep(dataFreq)
        return dataset, measurementCount