#ifndef quad
#define quad 
#include <iostream>
#include <stdio.h>
#include <wiringPiI2C.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <cmath>
#include <chrono>
#define Device_Address 0x68	/*Device Address/Identifier for MPU6050*/

#define PWR_MGMT_1   0x6B
#define SMPLRT_DIV   0x19
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define INT_ENABLE   0x38
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H  0x43
#define GYRO_YOUT_H  0x45
#define GYRO_ZOUT_H  0x47
#endif

using namespace std;
using namespace std::chrono;

float gx, gy, gz, ax,ay,az; 
float xmean = -0.0505, ymean = -0.3934, zmean = -2.19461, xstd = .007, ystd = .016, zstd =.014;
float X = xmean, Y = ymean, Z = 0;
float zupper = zstd*2, zlower = -zstd*2;
int count = 0;

//std functions

void MPU6050_Init();
short read_raw_data(int addr);
void ms_delay(int val);
int collect_data();
int zHold (int runTime);
int zPID (int curspeed, float *prevErr, float *intSum);
int fd = wiringPiI2CSetup(Device_Address);

int main() {
	
	MPU6050_Init();
	

	int time = 1;
	zHold(time*1000);
	return 0;
}

int zHold (int runTime) {
	int rollLen = 5;
	float runningValues [rollLen];
	float prevError = 0;
	float currentIntegralSum = 0;
	int timeSpent = 0;
	int curspeed = 1500;
	
	while (timeSpent < runTime) {
		auto start = high_resolution_clock::now();
		for (int i = 0; i < rollLen; i++){
			collect_data();
			//cout<< "velocity: " << velocity << "\tZ: " << Z << endl;
			//cout << az << endl;
			runningValues[count % rollLen] = az;
			//cout << runningValues[0] << "\t" << runningValues[1] <<"\t" << runningValues[2] <<"\t" << runningValues[3] <<"\t" << runningValues[4] <<endl;
			count++;
		}
		float runningSum = 0, avg = 0;
		for (int azVal = 0; azVal < rollLen; azVal++) {
			runningSum += runningValues[azVal];
		}
		avg = runningSum/rollLen;
		//cout << "avg:  " << avg << endl;
		
		if (avg > zupper || avg < zlower) {
				Z += avg;
				curspeed = zPID (curspeed, &prevError, &currentIntegralSum);
				//assign motor power here
			}
		auto stop = high_resolution_clock::now();
		auto duration = duration_cast<milliseconds>(stop-start);
		timeSpent += duration.count();
	}
	return 0;
}

int zPID (int curspeed, float *prevErr, float *intSum) {
	float kp = 1, ki = 1, kd = 1, integralThreshold = 50;
	float err = Z;
	float deriv = err - *prevErr;
	*intSum += err;
	float power = 0;
	
	power += kp * err;
	power += kd * deriv;
	
	if (abs(*intSum) >= integralThreshold) {
		power += ki * err;
	}
	
	curspeed += power;
	//cout << "gyro: \t" << Z << endl;
	//cout << "CurSpeed: " << curspeed << "  err: " << err << "  Deriv: " << deriv << "  TotSum: " << *intSum << endl;
	*prevErr = err;

	return curspeed;
}

int collect_data(){
	float Gyro_x,Gyro_y,Gyro_z;
	float Acc_x,Acc_y,Acc_z;                 
	
	Acc_x = read_raw_data(ACCEL_XOUT_H);
	Acc_y = read_raw_data(ACCEL_YOUT_H);
	Acc_z = read_raw_data(ACCEL_ZOUT_H);

	Gyro_x = read_raw_data(GYRO_XOUT_H);
	Gyro_y = read_raw_data(GYRO_YOUT_H);
	Gyro_z = read_raw_data(GYRO_ZOUT_H);
	
	ax = Acc_x/16384.0;
	ay = Acc_y/16384.0;
	az = Acc_z/8192.0 - zmean;
	gx = Gyro_x/131;
	gy = Gyro_y/131;
	gz = Gyro_z/131;
	//printf("\n Gx= %.3f \tGy= %.3f \tGz= %.3f\n",gx, gy, gz);
	//printf("\n sumx= %.3f \tsumy= %.3f \tsumz= %.3f\n",Xsum, Ysum, Zsum);
	//printf("\tAx=%.3f \tAy=%.3f \tAz=%.3f \n",ax,ay,az);
	
	return 0;
}

void MPU6050_Init(){
	
	wiringPiI2CWriteReg8 (fd, SMPLRT_DIV, 0x07);	/* Write to sample rate register */
	wiringPiI2CWriteReg8 (fd, PWR_MGMT_1, 0x01);	/* Write to power management register */
	wiringPiI2CWriteReg8 (fd, CONFIG, 0);		/* Write to Configuration register */
	wiringPiI2CWriteReg8 (fd, GYRO_CONFIG, 24);	/* Write to Gyro Configuration register */
	wiringPiI2CWriteReg8 (fd, INT_ENABLE, 0x01);	/*Write to interrupt enable register */
} 
	
short read_raw_data(int addr){
	short high_byte,low_byte,value;
	high_byte = wiringPiI2CReadReg8(fd, addr);
	low_byte = wiringPiI2CReadReg8(fd, addr+1);
	value = (high_byte << 8) | low_byte;
	return value;
}

void ms_delay(int val){
	int i,j;
	for(i=0;i<=val;i++)
		for(j=0;j<1200;j++);
}




