/*
 * tmp102.cpp
 *
 *  Created on: 15 gru 2018
 *      Author: kamil
 */

// add gpio controll and i2cdev driver

#include "tmp102.h"


tmp102::tmp102(uint8_t addr, char* i2c_bus){

	gpio4 = new GPIO("4");
	gpio4->setdir_gpio("out");
	gpio4->setval_gpio("1");
	addres = addr;

	sprintf(filename, i2c_bus);
		if ((file = open(filename, O_RDWR)) < 0) {
			printf("Failed to open the bus.");
			printf("error: %s (%d)\n", strerror(errno), errno);
			exit(1);
		}

		if (ioctl(file, I2C_SLAVE, addr) < 0) {
			printf("Failed to acquire bus access and/or talk to slave.\n");
			printf("error: %s (%d)\n", strerror(errno), errno);
			exit(1);
		}
		write(file, 0x00, 1);
}

float tmp102::readTemperature(){

	int i = 10;
	unsigned char buf[i];

	float data;

	// printf("Address: %x\n", addres);
	// Read 2 uint8 using I2C Read
	int k = read(file, buf, i);
	if ((k != i)) {
		printf("Amount of data read: %d\n", k);
		printf("error: %s (%d) %d\n", strerror(errno), errno,2);
	} else {
		printf("Data read:");
		for (int j = 0; j < i; j++){
			printf(" %x", buf[j]);
		}
		printf("\n");
		int temperature;
		temperature = ((buf[0] << 4) | (buf[1] >> 4));// + (1 << 8);
		printf("Temp pre scaling: %x\n", temperature);
		// temperature = ((buf[0]) << 8) | (buf[1]);
		// temperature >>= 4;

		if (temperature & (1 << 11))
			temperature |= 0xF800;

		data =  (float)temperature * 0.0625;
	}
	return data;
}



