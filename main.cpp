/*
 * main.cpp
 *
 *  Created on: 1 gru 2018
 *      Author: kamil
 */

#include "tmp102.h"
#include "lsm6ds3/LSM6DS3.h"
#include "bcm/bcm2835.h"
#include "l6470/include/l6470constants.h"
#include "l6470/include/motors.h"
#include "vl53l1x/VL53L1X.h"
#include <csignal>
#include <math.h>

#define GPIO_TOF_0 	RPI_V2_GPIO_P1_29
#define GPIO_TOF_1 	RPI_V2_GPIO_P1_31
#define GPIO_TOF_2 	RPI_V2_GPIO_P1_33
#define GPIO_TOF_3 	RPI_V2_GPIO_P1_35
#define GPIO_TOF_4	RPI_V2_GPIO_P1_37
#define GPIO_TOF_5 	RPI_V2_GPIO_P1_12
#define GPIO_TOF_6 	RPI_V2_GPIO_P1_16
#define GPIO_TOF_7 	RPI_V2_GPIO_P1_18
#define GPIO_TOF_8 	RPI_V2_GPIO_P1_32
#define GPIO_TOF_9 	RPI_V2_GPIO_P1_36
#define NDEBUG

void stepperTest();
void tofTest();
Motors *globalBoard;

void sigintHandler(int signum) {
	if (signum == SIGINT) {
		globalBoard->stop();
		exit(signum);
	}
}

int main(void) {
	signal(SIGINT, sigintHandler);

	if (bcm2835_init() == 0) {
			fprintf(stderr, "Not able to init the bmc2835 library\n");
			return -1;
	}


	LSM6DS3 SensorOne( I2C_MODE, 0x6B);

	if( SensorOne.begin() != 0 )
	{
		  printf("Problem starting the sensor \n");
	}
	else
	{
		  printf("Sensor with CS1 started.\n");
	}
	int i, n = 2000;
	// Acceleration
	float  ax, ay, az;
	// Raw and filtered gyro
	float gx, gy, gz, rgx, rgy, rgz, a = 0.5;
	gx = SensorOne.readFloatGyroX();
	gy = SensorOne.readFloatGyroY();
	gz = SensorOne.readFloatGyroZ();
	// Sum of gyro readings
	float sumgx = 0, sumgy = 0, sumgz = 0;
	// Tilt
	float ty;
	for(i=0; i<n; i++){
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
		rgx = SensorOne.readFloatGyroX();
		rgy = SensorOne.readFloatGyroY();
		rgz = SensorOne.readFloatGyroZ();
		gx = a*gx + (1-a)*rgx;
		gy = a*gy + (1-a)*rgy;
		gz = a*gz + (1-a)*rgz;
		sumgx += rgx;
		sumgy += rgy;
		sumgz += rgz;
		printf(" RawX = %f\n",rgx);
		printf(" RawY = %f\n",rgy);
		printf(" RawZ = %f\n",rgz);
		printf(" X1 = %f\n",gx);
		printf(" Y1 = %f\n",gy);
		printf(" Z1 = %f\n",gz);

		printf("\nTilt:\n");
		ty = atan2(ax,sqrt(ay*ay+az*az));
		printf(" Y = %f\n", ty);

		printf("\nThermometer:\n");
		printf(" Degrees C = %f\n",SensorOne.readTempC());

		printf("\nSensorOne Bus Errors Reported:\n");
		printf(" All '1's = %d\n",SensorOne.allOnesCounter);
		printf(" Non-success = %d\n",SensorOne.nonSuccessCounter);
		delay(100);
	}
	printf(" Average Gyro X = %f\n", sumgx/n);
	printf(" Average Gyro Y = %f\n", sumgy/n);
	printf(" Average Gyro Z = %f\n", sumgz/n);

	SensorOne.close_i2c();

	//stepperTest();

	//tmp102 czujnik(0x48,"/dev/i2c-0");
	//printf("Rys temperature: %f \n",czujnik.readTemperature());

	puts("Done!");

}

void tofTest(){

		uint16_t measurement[10];
		VL53L1X* sensor[10];
		sensor[0] = new VL53L1X(GPIO_TOF_0, VL53L1X::Medium);
		sensor[1] = new VL53L1X(GPIO_TOF_1, VL53L1X::Medium);
		sensor[2] = new VL53L1X(GPIO_TOF_2, VL53L1X::Medium);
		sensor[3] = new VL53L1X(GPIO_TOF_3, VL53L1X::Medium);
		sensor[4] = new VL53L1X(GPIO_TOF_4, VL53L1X::Medium);
		sensor[5] = new VL53L1X(GPIO_TOF_5, VL53L1X::Medium);
		sensor[6] = new VL53L1X(GPIO_TOF_6, VL53L1X::Medium);
		sensor[7] = new VL53L1X(GPIO_TOF_7, VL53L1X::Medium);
		sensor[8] = new VL53L1X(GPIO_TOF_8, VL53L1X::Medium);
		sensor[9] = new VL53L1X(GPIO_TOF_9, VL53L1X::Medium);


		while(1){
				for(int i=0; i<10; i++)
					measurement[i] = sensor[i]->read(0);
				printf("Distance: %d\n",measurement[0]);
				bcm2835_delay(150);
		}
}

void stepperTest(){

	long positionLeft,positionRight;
	int voltage;
	Motors board( BCM2835_SPI_CS0, GPIO_RESET_OUT);
	globalBoard = &board;
	board.setUp();
	board.setSpeed(50,50);
	bcm2835_delay(2000);
	positionLeft = board.getPositionLeft();
	positionRight = board.getPositionRight();
	voltage = board.getBatteryVoltage();
	printf("Absolute position: Left:%lu		Right%lu	Voltage:%d\n",positionLeft, positionRight,voltage);
	board.setSpeed(-50,-50);
	bcm2835_delay(2000);
	positionLeft = board.getPositionLeft();
	positionRight = board.getPositionRight();
	voltage = board.getBatteryVoltage();
	printf("Absolute position: Left:%lu		Right%lu	Voltage:%d\n",positionLeft, positionRight,voltage);
	board.setSpeed(30,30);
	bcm2835_delay(2000);
	positionLeft = board.getPositionLeft();
	positionRight = board.getPositionRight();
	voltage = board.getBatteryVoltage();
	printf("Absolute position: Left:%lu		Right%lu	Voltage:%d\n",positionLeft, positionRight,voltage);
	board.setSpeed(-30,-30);
	bcm2835_delay(2000);
	positionLeft = board.getPositionLeft();
	positionRight = board.getPositionRight();
	voltage = board.getBatteryVoltage();
	printf("Absolute position: Left:%lu		Right%lu	Voltage:%d\n",positionLeft, positionRight,voltage);
	board.setSpeed(20,-20);
	bcm2835_delay(1000);
	positionLeft = board.getPositionLeft();
	positionRight = board.getPositionRight();
	voltage = board.getBatteryVoltage();
	printf("Absolute position: Left:%lu		Right%lu	Voltage:%d\n",positionLeft, positionRight,voltage);
	board.setSpeed(-20,20);
	bcm2835_delay(3000);
	positionLeft = board.getPositionLeft();
	positionRight = board.getPositionRight();
	voltage = board.getBatteryVoltage();
	printf("Absolute position: Left:%lu		Right%lu	Voltage:%d\n",positionLeft, positionRight,voltage);
	board.setSpeed(100,100);
	bcm2835_delay(1000);
	positionLeft = board.getPositionLeft();
	positionRight = board.getPositionRight();
	voltage = board.getBatteryVoltage();
	printf("Absolute position: Left:%lu		Right%lu	Voltage:%d\n",positionLeft, positionRight,voltage);
	board.setSpeed(-200,-200);
	bcm2835_delay(1000);
	positionLeft = board.getPositionLeft();
	positionRight = board.getPositionRight();
	voltage = board.getBatteryVoltage();
	printf("Absolute position: Left:%lu		Right%lu	Voltage:%d\n",positionLeft, positionRight,voltage);
	board.stop();

	////////////////////////////////////////////////////////////////////////////////////////
}

