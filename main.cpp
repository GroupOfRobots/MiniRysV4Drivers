/*
 * main.cpp
 *
 *  Created on: 1 gru 2018
 *      Author: kamil
 */

#include "tmp102.h"
#include "lsm6ds3/LSM6DS3.h"
#include "bcm/bcm2835.h"
#include "l6470/include/autodriver.h"
#include "l6470/include/l6470constants.h"
#include "vl53l1x/VL53L1X.h"

#define GPIO_BUSY_IN	RPI_V2_GPIO_P1_13
#define GPIO_RESET_OUT 	RPI_V2_GPIO_P1_15

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
	for(i=0; i<20; i++){
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

	unsigned long temp;

	bcm2835_spi_begin();

	bcm2835_gpio_fsel(GPIO_RESET_OUT, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_set(GPIO_RESET_OUT);

	bcm2835_gpio_fsel(GPIO_BUSY_IN, BCM2835_GPIO_FSEL_INPT);

	bcm2835_gpio_clr(GPIO_RESET_OUT);
	bcm2835_delayMicroseconds(10000);
	bcm2835_gpio_set(GPIO_RESET_OUT);
	bcm2835_delayMicroseconds(10000);

	AutoDriver board(0, BCM2835_SPI_CS0, GPIO_RESET_OUT);
	AutoDriver board1(1, BCM2835_SPI_CS0, GPIO_RESET_OUT);

	/////////////////////////////////////TESTY//////////////////////////////////////////////

	// first check  board config register, should be 0x2E88 on bootup
	temp = board.getParam(L6470_PARAM_CONFIG);
	printf("Config reg value board0: %4x\n", (int) temp);
	temp = board1.getParam(L6470_PARAM_CONFIG);
	printf("Config reg valueboard1: %4x\n", (int) temp);

	// Now check the status of the board. Should be 0x7c03
	temp = board.getStatus();
	printf("Status reg value board0: %4x\n", (int) temp);
	temp = board1.getStatus();
	printf("Status reg value board1: %4x\n", (int) temp);

	board.configStepMode(0x05);   // 0 microsteps per step
	board.setMaxSpeed(300);        // 10000 steps/s max
	board.setMinSpeed(10);        // 10 steps/s min
	board.setFullSpeed(600);       // microstep below 10000 steps/s
	board.setAcc(100);             // accelerate at 10000 steps/s/s
	board.setDec(100);
	board.setPWMFreq((0x00)<<13, (0x07)<<10); // 62.5kHz PWM freq
	board.setSlewRate(0x0300);   // Upping the edge speed increases torque.
	board.setOCThreshold(0x08);  // OC threshold 3000mA
	board.setOCShutdown(0x0000); // don't shutdown on OC
	board.setVoltageComp(0x0000); // don't compensate for motor V
	board.setSwitchMode(0x0010);    // Switch is not hard stop
	board.setAccKVAL(255);           // We'll tinker with these later, if needed.
	board.setDecKVAL(255);
	board.setRunKVAL(255);
	board.setHoldKVAL(32);           // This controls the holding current; keep it low.

	board1.configStepMode(0x05);   // 0 microsteps per step
	board1.setMaxSpeed(300);        // 10000 steps/s max
	board1.setMinSpeed(10);        // 10 steps/s min
	board1.setFullSpeed(600);       // microstep below 10000 steps/s
	board1.setAcc(100);             // accelerate at 10000 steps/s/s
	board1.setDec(100);
	board1.setPWMFreq((0x00)<<13, (0x07)<<10); // 62.5kHz PWM freq
	board1.setSlewRate(0x0300);   // Upping the edge speed increases torque.
	board1.setOCThreshold(0x08);  // OC threshold 3000mA
	board1.setOCShutdown(0x0000); // don't shutdown on OC
	board1.setVoltageComp(0x0000); // don't compensate for motor V
	board1.setSwitchMode(0x0010);    // Switch is not hard stop
	board1.setAccKVAL(255);           // We'll tinker with these later, if needed.
	board1.setDecKVAL(255);
	board1.setRunKVAL(255);
	board1.setHoldKVAL(32);           // This controls the holding current; keep it low.




	////////////////////////////////////////////////////////////////////////////////////////

	while (board.busyCheck())
		;
	while (board1.busyCheck())
		;
	board1.run(1,200);
	board.run(1,200);

	bcm2835_delay(3000);

	board.softStop();
	board1.softStop();

	while (board.busyCheck())
		;

	board.hardHiZ();

	board1.hardHiZ();
}

