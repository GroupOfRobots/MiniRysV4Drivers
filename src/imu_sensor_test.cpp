#include <stdio.h>
#include <fstream>
#include <csignal>
#include <math.h>
#include "bcm/bcm2835.h"
#include "lsm6ds3/LSM6DS3.h"
#include "lsm6ds3/filter.h"

bool destroy = false;

void sigintHandler(int signum) {
	if (signum == SIGINT) {
		destroy = true;
	}
}

int main(void)
{
	signal(SIGINT, sigintHandler);

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
			if (destroy) {
				SensorOne.close_i2c();
				return 0;
			}
			  printf("%d\n",i);
			  delay(1000);
		  }
	}

	// Acceleration
	float  ax, ay, az;
	ax = SensorOne.readFloatAccelX();
	ay = SensorOne.readFloatAccelY();
	az = SensorOne.readFloatAccelZ();

	// Gyro
	float rgx, rgy, rgz;
	const float offX = 3.755909, offY = -5.435584, offZ = -3.461446;
	const float offXRad = -0.000329, offYRad = -0.000224, offZRad = 0.000880;
	rgx = (SensorOne.readFloatGyroX() - offX)*M_PI/180 - offXRad;
	rgy = (SensorOne.readFloatGyroY() - offY)*M_PI/180 - offYRad;
	rgz = (SensorOne.readFloatGyroZ() - offZ)*M_PI/180 - offZRad;

	// Tilt
	float ty = atan2(ax,sqrt(ay*ay+az*az));

	// Filtering
	float param = 0.05;
	float freq = 10;
	filter f(ty,param,freq);
	float f_gy, f_ty;

	while(!destroy) {
		printf("\033[H\033[J");
		//Get all parameters
		printf("\nAccelerometer:\n");
		ax = SensorOne.readFloatAccelX();
		ay = SensorOne.readFloatAccelY();
		az = SensorOne.readFloatAccelZ();
		printf(" X = %f\n",ax);
		printf(" Y = %f\n",ay);
		printf(" Z = %f\n",az);
		printf(" ACC = %f\n", sqrt(ax*ax+ay*ay+az*az));

		printf("\nGyroscope:\n");
		rgx = (SensorOne.readFloatGyroX() - offX)*M_PI/180 - offXRad;
		rgy = (SensorOne.readFloatGyroY() - offY)*M_PI/180 - offYRad;
		rgz = (SensorOne.readFloatGyroZ() - offZ)*M_PI/180 - offZRad;
		printf(" X = %f\n",rgx);
		printf(" Y = %f",rgy);
		printf(" Z = %f\n",rgz);

		printf("\nTilt:\n");
		ty = atan2(ax,sqrt(ay*ay+az*az));
		printf(" Y = %f", ty);

		printf("\nAfter filtering:\n");
		f_ty = f.getAngle(ty,rgy);
		f_gy = f.getGyro();
		printf(" Gyro Y: %f", f_gy);
		printf(" Tilt Y: %f\n", f_ty);

		printf("\nThermometer:\n");
		printf(" Degrees C = %f\n",SensorOne.readTempC());

		printf("\nSensorOne Bus Errors Reported:\n");
		printf(" All '1's = %d\n",SensorOne.allOnesCounter);
		printf(" Non-success = %d\n",SensorOne.nonSuccessCounter);

		delay(1000/freq);
	}

	SensorOne.close_i2c();

	return 0;
}