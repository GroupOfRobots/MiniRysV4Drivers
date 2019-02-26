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

void stepperTest();
void tofTest();

int main(void) {

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
	int i;
	/*for(i=0; i<20; i++){
		//Get all parameters
		printf("\nAccelerometer:\n");
		printf(" X1 = %f\n",SensorOne.readFloatAccelX());
		printf(" Y1 = %f\n",SensorOne.readFloatAccelY());
		printf(" Z1 = %f\n",SensorOne.readFloatAccelZ());

		printf("\nGyroscope:\n");
		printf(" X1 = %f\n",SensorOne.readFloatGyroX());
		printf(" Y1 = %f\n",SensorOne.readFloatGyroY());
		printf(" Z1 = %f\n",SensorOne.readFloatGyroZ());

		printf("\nThermometer:\n");
		printf(" Degrees C = %f\n",SensorOne.readTempC());

		printf("\nSensorOne Bus Errors Reported:\n");
		printf(" All '1's = %d\n",SensorOne.allOnesCounter);
		printf(" Non-success = %d\n",SensorOne.nonSuccessCounter);
		delay(100);
	}
	*/
	SensorOne.close_i2c();

	stepperTest();

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

	Motors board( BCM2835_SPI_CS0, GPIO_RESET_OUT);
	board.setUp();
	board.setSpeed(200,200);
	bcm2835_delay(1000);
	board.setSpeed(100,50);
	bcm2835_delay(1500);
	board.setSpeed(200,200);
	bcm2835_delay(1000);
	board.setSpeed(50,100);
	bcm2835_delay(1500);
	board.setSpeed(200,200);
	bcm2835_delay(1000);
	board.setSpeed(100,50);
	bcm2835_delay(1500);
	board.setSpeed(200,200);
	bcm2835_delay(1000);
	board.setSpeed(50,100);
	bcm2835_delay(1500);
	board.stop();

	////////////////////////////////////////////////////////////////////////////////////////
}

