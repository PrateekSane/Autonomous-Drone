#include <iostream>
#include <stdio.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <pigpio.h>

using namespace std;
void ms_delay(int val);
//int res = wiringPiSetupGpio();
int res = gpioInitialise();
int main () {
	int motors[4] = {13, 5, 16, 12};
	cout << res << endl;
	
	for(int i = 0; i < 4; i++){
		gpioSetMode(motors[i], PI_OUTPUT);
		gpioServo(motors[i], 1000);
	}
	
	for(int i = 0; i < 4; i++){
		gpioServo(motors[i], 2000);
	}
	ms_delay(300);
	while(true) {
		for(int i = 0; i < 4; i++){
			gpioServo(motors[i], 1000);
		}
	}
	gpioTerminate();
	
	

return 1;
}

void ms_delay(int val){
	int i,j;
	for(i=0;i<=val;i++)
		for(j=0;j<1200000;j++);
}
