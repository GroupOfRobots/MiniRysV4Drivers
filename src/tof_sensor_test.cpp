#include <stdio.h>
#include <fstream>
#include <csignal>
#include <math.h>
#include "bcm/bcm2835.h"
#include "vl53l1x/VL53L1X.h"

#define GPIO_TOF_1 	RPI_V2_GPIO_P1_12
#define GPIO_TOF_2 	RPI_V2_GPIO_P1_16
#define GPIO_TOF_3 	RPI_V2_GPIO_P1_18
#define GPIO_TOF_4 	RPI_V2_GPIO_P1_29
#define GPIO_TOF_5	RPI_V2_GPIO_P1_32
#define GPIO_TOF_6 	RPI_V2_GPIO_P1_31
#define GPIO_TOF_7 	RPI_V2_GPIO_P1_33
#define GPIO_TOF_8 	RPI_V2_GPIO_P1_35
#define GPIO_TOF_9 	RPI_V2_GPIO_P1_36
#define GPIO_TOF_10 RPI_V2_GPIO_P1_37

const int numOfSensors = 8;
const int period_ms = 100;
bool endProcess = false;

void sigintHandler(int signum) {
	if (signum == SIGINT) {
		endProcess = true;
	}
}

int main(void){
	signal(SIGINT, sigintHandler);

	if (bcm2835_init() == 0) {
		fprintf(stderr, "Not able to init the bmc2835 library\n");
		return -1;
	}

	VL53L1X *globalSensors[numOfSensors];
	uint16_t measurement[numOfSensors];

	bcm2835_gpio_fsel(GPIO_TOF_1, BCM2835_GPIO_FSEL_OUTP);
	// bcm2835_gpio_fsel(GPIO_TOF_2, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_3, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_4, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_5, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_6, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_7, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_8, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_9, BCM2835_GPIO_FSEL_OUTP);
	// bcm2835_gpio_fsel(GPIO_TOF_10, BCM2835_GPIO_FSEL_OUTP);

	//disable all sensors first
	bcm2835_gpio_clr(GPIO_TOF_1); // górny
	// bcm2835_gpio_clr(GPIO_TOF_2); // górny
	bcm2835_gpio_clr(GPIO_TOF_3);
	bcm2835_gpio_clr(GPIO_TOF_4);
	bcm2835_gpio_clr(GPIO_TOF_5);
	bcm2835_gpio_clr(GPIO_TOF_6);
	bcm2835_gpio_clr(GPIO_TOF_7);
	bcm2835_gpio_clr(GPIO_TOF_8);
	bcm2835_gpio_clr(GPIO_TOF_9);
	// bcm2835_gpio_clr(GPIO_TOF_10);

	bcm2835_i2c_begin(); //begin I2C
	bcm2835_i2c_set_baudrate(40000);

	//enable sensor one and change address
	bcm2835_gpio_set(GPIO_TOF_1);
	delay(10);
	globalSensors[0] = new VL53L1X(VL53L1X::Medium,0x29);
	delay(10);
	globalSensors[0]->setAddress(0x30);
	delay(10);
	puts("Sensor one started at: 0x30");

	// bcm2835_gpio_set(GPIO_TOF_2);
	bcm2835_gpio_set(GPIO_TOF_9);
	delay(10);
	globalSensors[1] = new VL53L1X(VL53L1X::Medium,0x29);
	delay(10);
	globalSensors[1]->setAddress(0x31);
	delay(10);
	puts("Sensor two started at: 0x31");

	bcm2835_gpio_set(GPIO_TOF_3);
	delay(10);
	globalSensors[2] = new VL53L1X(VL53L1X::Medium,0x29);
	delay(10);
	globalSensors[2]->setAddress(0x32);
	delay(10);
	puts("Sensor three started at: 0x32");

	bcm2835_gpio_set(GPIO_TOF_4);
	delay(10);
	globalSensors[3] = new VL53L1X(VL53L1X::Medium,0x29);
	delay(10);
	globalSensors[3]->setAddress(0x33);
	delay(10);
	puts("Sensor four started at: 0x33");

	bcm2835_gpio_set(GPIO_TOF_5);
	delay(10);
	globalSensors[4] = new VL53L1X(VL53L1X::Medium,0x29);
	delay(10);
	globalSensors[4]->setAddress(0x34);
	delay(10);
	puts("Sensor five started at: 0x34");

	bcm2835_gpio_set(GPIO_TOF_6);
	delay(10);
	globalSensors[5] = new VL53L1X(VL53L1X::Medium,0x29);
	delay(10);
	globalSensors[5]->setAddress(0x35);
	delay(10);
	puts("Sensor six started at: 0x35");

	bcm2835_gpio_set(GPIO_TOF_7);
	delay(10);
	globalSensors[6] = new VL53L1X(VL53L1X::Medium,0x29);
	delay(10);
	globalSensors[6]->setAddress(0x36);
	delay(10);
	puts("Sensor seven started at: 0x36");

	bcm2835_gpio_set(GPIO_TOF_8);
	delay(10);
	globalSensors[7] = new VL53L1X(VL53L1X::Medium,0x29);
	delay(10);
	globalSensors[7]->setAddress(0x37);
	delay(10);
	puts("Sensor eight started at: 0x37");

	// bcm2835_gpio_set(GPIO_TOF_9);
	// delay(10);
	// globalSensors[8] = new VL53L1X(VL53L1X::Medium,0x29);
	// delay(10);
	// globalSensors[8]->setAddress(0x38);
	// delay(10);
	// puts("Sensor nine started at: 0x38");

	// bcm2835_gpio_set(GPIO_TOF_10);
	// delay(10);
	// globalSensors[9] = new VL53L1X(VL53L1X::Medium,0x29);
	// delay(10);
	// globalSensors[9]->setAddress(0x39);
	// delay(10);
	// puts("Sensor ten started at: 0x39");


	for(int i = 0; i < numOfSensors; i++){
		globalSensors[i]->startContinuous(period_ms);
		delay(10);
	}

	while(!endProcess){
		printf("\n");
		printf("\033[H\033[J");
		for(int i = 0; i < numOfSensors; i++){
			measurement[i] = globalSensors[i]->readData(1);
			printf("%d:%5d ",i,measurement[i]);
		}
		delay(period_ms);
	}

	for(int i = 0; i < numOfSensors; i++){
		globalSensors[i]->disable();
		delay(10);
	}

	return 0;
}