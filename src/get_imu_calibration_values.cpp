#include <stdio.h>
#include <fstream>
#include <math.h>
#include <iostream>
#include "bcm/bcm2835.h"
#include "lsm6ds3/LSM6DS3.h"

int main(void)
{
	if (bcm2835_init() == 0) {
		fprintf(stderr, "Not able to init the bmc2835 library\n");
		return -1;
	}

	LSM6DS3 SensorOne( I2C_MODE, 0x6B);

	int status = SensorOne.begin();
	if( status != 0 )
	{
		printf("Problem number %d starting the sensor \n", status);
		return 0;
	}
	else
	{
		printf("Sensor with CS1 started, awaiting calibration.\n");
		for (int i = 10; i>0; i--){
			printf("%d\n",i);
			delay(1000);
		}
	}

	// Acceleration
	float  ax = 0.0, ay = 0.0, az = 0.0;

	// Gyro
	float rgx = 0.0, rgy = 0.0, rgz = 0.0;

	// Frequency
	float frequency = 100;
	int readings = 10000;

	printf("Place robot on the support. Press any key when ready...");
	getchar();

	for (int i = 0; i < readings; i++){
		ax += SensorOne.readFloatAccelX()/readings/4;
		ay += SensorOne.readFloatAccelY()/readings/4;
		az += SensorOne.readFloatAccelZ()/readings/4;
		rgx += SensorOne.readFloatGyroX()/readings/4;
		rgy += SensorOne.readFloatGyroY()/readings/4;
		rgz += SensorOne.readFloatGyroZ()/readings/4;
		delay(1000/frequency);
	}

	printf("\nRotate the robot 90 degrees right. Press any key when ready...");
	getchar();

	for (int i = 0; i < readings; i++){
		ax += SensorOne.readFloatAccelX()/readings/4;
		ay += SensorOne.readFloatAccelY()/readings/4;
		az += SensorOne.readFloatAccelZ()/readings/4;
		rgx += SensorOne.readFloatGyroX()/readings/4;
		rgy += SensorOne.readFloatGyroY()/readings/4;
		rgz += SensorOne.readFloatGyroZ()/readings/4;
		delay(1000/frequency);
	}

	printf("\nRotate the robot 90 degrees right. Press any key when ready...");
	getchar();

	for (int i = 0; i < readings; i++){
		ax += SensorOne.readFloatAccelX()/readings/4;
		ay += SensorOne.readFloatAccelY()/readings/4;
		az += SensorOne.readFloatAccelZ()/readings/4;
		rgx += SensorOne.readFloatGyroX()/readings/4;
		rgy += SensorOne.readFloatGyroY()/readings/4;
		rgz += SensorOne.readFloatGyroZ()/readings/4;
		delay(1000/frequency);
	}

	printf("\nRotate the robot 90 degrees right. Press any key when ready...");
	getchar();

	for (int i = 0; i < readings; i++){
		ax += SensorOne.readFloatAccelX()/readings/4;
		ay += SensorOne.readFloatAccelY()/readings/4;
		az += SensorOne.readFloatAccelZ()/readings/4;
		rgx += SensorOne.readFloatGyroX()/readings/4;
		rgy += SensorOne.readFloatGyroY()/readings/4;
		rgz += SensorOne.readFloatGyroZ()/readings/4;
		delay(1000/frequency);
	}

	printf("\nReadings complete.\nMean accelerometer values(x,y,z) are: \t%f\t%f\t%f\nGravity: %f\nTilt: %f\nMean gyro values(x,y,z) are: \t\t%f\t%f\t%f\n", ax, ay, az, sqrt(ax*ax+ay*ay+az*az), atan2(ax, sqrt(ay*ay + az*az)), rgx, rgy, rgz);

	SensorOne.close_i2c();

	return 0;
}