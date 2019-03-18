/*
 * main.cpp
 *
 *  Created on: 1 gru 2018
 *      Author: kamil
 */

#include "tmp102.h"
#include "lsm6ds3/LSM6DS3.h"
#include "lsm6ds3/filter.h"
#include "bcm/bcm2835.h"
#include "l6470/include/l6470constants.h"
#include "l6470/include/motors.h"
#include "vl53l1x/VL53L1X.h"
#include <csignal>
#include <math.h>
#include <fstream>

#define GPIO_TOF_1 	RPI_V2_GPIO_P1_12
#define GPIO_TOF_2 	RPI_V2_GPIO_P1_16
#define GPIO_TOF_3 	RPI_V2_GPIO_P1_18
#define GPIO_TOF_4 	RPI_V2_GPIO_P1_29
#define GPIO_TOF_5	RPI_V2_GPIO_P1_32
#define GPIO_TOF_6 	RPI_V2_GPIO_P1_31
#define GPIO_TOF_7 	RPI_V2_GPIO_P1_33
#define GPIO_TOF_8 	RPI_V2_GPIO_P1_35
#define GPIO_TOF_9 	RPI_V2_GPIO_P1_36
#define GPIO_TOF_10 	RPI_V2_GPIO_P1_37
#define NDEBUG

void stepperTest();
void tofTest();
void IMUtest();
Motors *globalBoard;
VL53L1X *globalSensors[2];

void sigintHandler(int signum) {
	if (signum == SIGINT) {
		globalBoard->stop();
		globalSensors[0]->disable();
		globalSensors[1]->disable();
		exit(signum);
	}
}

int main(void) {
	signal(SIGINT, sigintHandler);

	if (bcm2835_init() == 0) {
			fprintf(stderr, "Not able to init the bmc2835 library\n");
			return -1;
	}



	//tofTest();
	//IMUtest();
	stepperTest();




	//stepperTest();

	puts("Done!");

}

void tofTest(){

		//disable all sensors first
		bcm2835_gpio_fsel(GPIO_TOF_1, BCM2835_GPIO_FSEL_OUTP);
		bcm2835_gpio_clr(GPIO_TOF_1);
		bcm2835_gpio_fsel(GPIO_TOF_2, BCM2835_GPIO_FSEL_OUTP);
		bcm2835_gpio_clr(GPIO_TOF_2);
		bcm2835_gpio_fsel(GPIO_TOF_3, BCM2835_GPIO_FSEL_OUTP);
		bcm2835_gpio_clr(GPIO_TOF_3);
		bcm2835_gpio_fsel(GPIO_TOF_4, BCM2835_GPIO_FSEL_OUTP);
		bcm2835_gpio_clr(GPIO_TOF_4);
		bcm2835_gpio_fsel(GPIO_TOF_5, BCM2835_GPIO_FSEL_OUTP);
		bcm2835_gpio_clr(GPIO_TOF_5);
		bcm2835_gpio_fsel(GPIO_TOF_6, BCM2835_GPIO_FSEL_OUTP);
		bcm2835_gpio_clr(GPIO_TOF_6);
		bcm2835_gpio_fsel(GPIO_TOF_7, BCM2835_GPIO_FSEL_OUTP);
		bcm2835_gpio_clr(GPIO_TOF_7);
		bcm2835_gpio_fsel(GPIO_TOF_8, BCM2835_GPIO_FSEL_OUTP);
		bcm2835_gpio_clr(GPIO_TOF_8);
		bcm2835_gpio_fsel(GPIO_TOF_9, BCM2835_GPIO_FSEL_OUTP);
		bcm2835_gpio_clr(GPIO_TOF_9);
		bcm2835_gpio_fsel(GPIO_TOF_10, BCM2835_GPIO_FSEL_OUTP);
		bcm2835_gpio_set(GPIO_TOF_10);

		bcm2835_i2c_begin(); //begin I2C
		bcm2835_i2c_set_baudrate(10000);

		puts("all sensors powered down");

		bcm2835_delay(100);
		uint16_t measurement[10];


		//enable sensor one and change address
		bcm2835_gpio_set(GPIO_TOF_1);
		delay(100);
		globalSensors[0] = new VL53L1X(VL53L1X::Medium,0x29);
		delay(100);
		globalSensors[0]->setAddress(0x20);
		delay(100);
		puts("Sensor one started at addr: 0x20");

		bcm2835_gpio_set(GPIO_TOF_2);
		delay(100);
		globalSensors[1] = new VL53L1X(VL53L1X::Medium,0x29);
		delay(100);
		globalSensors[1]->setAddress(0x21);
		delay(100);
		puts("Sensor one started at addr: 0x21");

		bcm2835_gpio_set(GPIO_TOF_3);
		delay(100);
		globalSensors[2] = new VL53L1X(VL53L1X::Medium,0x29);
		delay(100);
		globalSensors[2]->setAddress(0x22);
		delay(100);
		puts("Sensor one started at addr: 0x22");




		globalSensors[0]->startContinuous(100);
		globalSensors[1]->startContinuous(100);
		globalSensors[2]->startContinuous(100);

		for(int i=0; i<10; i++){
					measurement[0] = globalSensors[0]->readData(0);
					measurement[1] = globalSensors[1]->readData(0);
					measurement[2] = globalSensors[2]->readData(0);
					printf("Distance sensor1: %d: sensor2: %d sensor3: %d\n",measurement[0],measurement[1],measurement[2]);
					bcm2835_delay(100);
		}
		globalSensors[0]->disable();
		globalSensors[1]->disable();
		globalSensors[2]->disable();
		//bcm2835_i2c_end();
}

void stepperTest(){

	long positionLeft,positionRight;
	int voltage;
	Motors board( BCM2835_SPI_CS0, GPIO_RESET_OUT);
	globalBoard = &board;
	board.setUp();
	positionLeft = board.getPositionLeft();
	positionRight = board.getPositionRight();
	voltage = board.getBatteryVoltage();
	printf("Absolute position: Left:%lu		Right:%lu	Voltage:%d\n",positionLeft, positionRight,voltage);
//	board.setSpeed(50,50);
//	bcm2835_delay(5000);
//	positionLeft = board.getPositionLeft();
//	positionRight = board.getPositionRight();
//	voltage = board.getBatteryVoltage();
//	printf("Absolute position: Left:%lu		Right:%lu	Voltage:%d\n",positionLeft, positionRight,voltage);
//	board.setSpeed(-50,-50);
//	bcm2835_delay(5000);
//	positionLeft = board.getPositionLeft();
//	positionRight = board.getPositionRight();
//	voltage = board.getBatteryVoltage();
//	printf("Absolute position: Left:%lu		Right:%lu	Voltage:%d\n",positionLeft, positionRight,voltage);
//	board.setSpeed(30,30);
//	bcm2835_delay(5000);
//	positionLeft = board.getPositionLeft();
//	positionRight = board.getPositionRight();
//	voltage = board.getBatteryVoltage();
//	printf("Absolute position: Left:%lu		Right:%lu	Voltage:%d\n",positionLeft, positionRight,voltage);
//	board.setSpeed(-30,-30);
//	bcm2835_delay(5000);
//	positionLeft = board.getPositionLeft();
//	positionRight = board.getPositionRight();
//	voltage = board.getBatteryVoltage();
//	printf("Absolute position: Left:%lu		Right:%lu	Voltage:%d\n",positionLeft, positionRight,voltage);
//	board.setSpeed(20,-20);
//	bcm2835_delay(5000);
//	positionLeft = board.getPositionLeft();
//	positionRight = board.getPositionRight();
//	voltage = board.getBatteryVoltage();
//	printf("Absolute position: Left:%lu		Right:%lu	Voltage:%d\n",positionLeft, positionRight,voltage);
//	board.setSpeed(-20,20);
//	bcm2835_delay(5000);
//	positionLeft = board.getPositionLeft();
//	positionRight = board.getPositionRight();
//	voltage = board.getBatteryVoltage();
//	printf("Absolute position: Left:%lu		Right:%lu	Voltage:%d\n",positionLeft, positionRight,voltage);
//	board.setSpeed(200,200);
//	bcm2835_delay(5000);
//	positionLeft = board.getPositionLeft();
//	positionRight = board.getPositionRight();
//	voltage = board.getBatteryVoltage();
//	printf("Absolute position: Left:%lu		Right:%lu	Voltage:%d\n",positionLeft, positionRight,voltage);
//	board.setSpeed(-200,-200);
//	bcm2835_delay(5000);
//	positionLeft = board.getPositionLeft();
//	positionRight = board.getPositionRight();
//	voltage = board.getBatteryVoltage();
//	printf("Absolute position: Left:%lu		Right:%lu	Voltage:%d\n",positionLeft, positionRight,voltage);
	board.setSpeed(320,320);
	bcm2835_delay(30000);
	positionLeft = board.getPositionLeft();
	positionRight = board.getPositionRight();
	voltage = board.getBatteryVoltage();
	printf("Absolute position: Left:%lu		Right:%lu	Voltage:%d\n",positionLeft, positionRight,voltage);
	board.stop();

	////////////////////////////////////////////////////////////////////////////////////////
}

void IMUtest(){
	LSM6DS3 SensorOne( I2C_MODE, 0x6B);

	if( SensorOne.begin() != 0 )
	{
		  printf("Problem starting the sensor \n");
	}
	else
	{
		  printf("Sensor with CS1 started.\n");
	}
	int i, n = 1000;

	// Acceleration
	float  ax, ay, az;
	ax = SensorOne.readFloatAccelX();
	ay = SensorOne.readFloatAccelY();
	az = SensorOne.readFloatAccelZ();

	// Gyro
	float rgx, rgy, rgz;
	const float offX = 3.597539, offY = -5.142877, offZ = -3.623744;
	rgx = (SensorOne.readFloatGyroX() - offX)*M_PI/180;
	rgy = (SensorOne.readFloatGyroY() - offY)*M_PI/180;
	rgz = (SensorOne.readFloatGyroZ() - offZ)*M_PI/180;

	// Sum of gyro readings
	float sumgx = 0, sumgy = 0, sumgz = 0;

	// Tilt
	float ty = atan2(ax,sqrt(ay*ay+az*az));

	// Filtering
	filter f(ty,0.5,10);
	float f_gy, f_ty;

	// Log file
	std::ofstream file;
	file.open("imu_log");

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
		rgx = (SensorOne.readFloatGyroX() - offX)*M_PI/180;
		rgy = (SensorOne.readFloatGyroY() - offY)*M_PI/180;
		rgz = (SensorOne.readFloatGyroZ() - offZ)*M_PI/180;
		sumgx += rgx;
		sumgy += rgy;
		sumgz += rgz;
		printf(" X = %f\n",rgx);
		printf(" Y = %f\n",rgy);
		printf(" Z = %f\n",rgz);

		printf("\nTilt:\n");
		ty = atan2(ax,sqrt(ay*ay+az*az));
		printf(" Y = %f\n", ty);

		printf("\nAfter filtering:\n");
		f_ty = f.getAngle(ty,rgy);
		f_gy = f.getGyro();
		printf(" Gyro Y: %f\n", f_gy);
		printf(" Tilt Y: %f\n", f_ty);

		printf("\nThermometer:\n");
		printf(" Degrees C = %f\n",SensorOne.readTempC());

		printf("\nSensorOne Bus Errors Reported:\n");
		printf(" All '1's = %d\n",SensorOne.allOnesCounter);
		printf(" Non-success = %d\n",SensorOne.nonSuccessCounter);

		//Write to file
		file << i*0.1 << " " << rgy << " " << ty << " " << f_gy << " " << f_ty << std::endl;

		delay(100);
	}
	file.close();

	printf(" Average Gyro X = %f\n", sumgx/n);
	printf(" Average Gyro Y = %f\n", sumgy/n);
	printf(" Average Gyro Z = %f\n", sumgz/n);

	SensorOne.close_i2c();
}

void TMPtest(){ //
	tmp102 czujnik(0x48,"/dev/i2c-0");
	printf("Rys temperature: %f \n",czujnik.readTemperature());

}

