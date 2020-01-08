#include <stdio.h>
#include <fstream>
#include <csignal>
#include "bcm/bcm2835.h"
#include "l6470/motors.h"

bool endProcess = false;

void sigintHandler(int signum) {
	if (signum == SIGINT) {
		endProcess = true;
	}
}

int main(void)
{
	signal(SIGINT, sigintHandler);

	if (bcm2835_init() == 0) {
			fprintf(stderr, "Not able to init the bmc2835 library\n");
			return -1;
	}

	long positionLeft, positionRight;
	Motors board( BCM2835_SPI_CS0, GPIO_RESET_OUT);
	board.setUp();
	board.resetPosition();
	positionLeft = board.getPositionLeft();
	positionRight = board.getPositionRight();
	printf("Absolute position: Left:%lu		Right:%lu \n",positionLeft, positionRight);

	board.setSpeed(100,100);
	delay(500);
	board.setSpeed(0, 0);
	delay(500);
	positionLeft = board.getPositionLeft();
	positionRight = board.getPositionRight();
	printf("Absolute position: Left:%lu		Right:%lu \n",positionLeft, positionRight);

	board.setSpeed(-100,-100);
	delay(500);
	board.setSpeed(0, 0);
	delay(500);
	positionLeft = board.getPositionLeft();
	positionRight = board.getPositionRight();
	printf("Absolute position: Left:%lu		Right:%lu \n",positionLeft, positionRight);

	board.setSpeed(100,-100);
	delay(500);
	board.setSpeed(0, 0);
	delay(500);
	positionLeft = board.getPositionLeft();
	positionRight = board.getPositionRight();
	printf("Absolute position: Left:%lu		Right:%lu \n",positionLeft, positionRight);

	board.setSpeed(-100,100);
	delay(500);
	board.setSpeed(0, 0);
	delay(500);
	positionLeft = board.getPositionLeft();
	positionRight = board.getPositionRight();
	printf("Absolute position: Left:%lu		Right:%lu \n",positionLeft, positionRight);

	board.stop();
	return 0;
}