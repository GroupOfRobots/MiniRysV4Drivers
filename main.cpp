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
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

#define JOY_DEV "/dev/input/js0"

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
#define NDEBUG

void stepperTest();
void tofTest();
void IMUtest();
void distanceTest();
void joyControl();
Motors *globalBoard;
VL53L1X *globalSensors[10];
uint16_t measurement[10];

void sigintHandler(int signum) {
	if (signum == SIGINT) {
		globalBoard->Dump();
		globalBoard->stop();
		for(int i=0; i<10; i++)
			globalSensors[i]->disable();
		exit(signum);
	}
}

int main(void) {
	signal(SIGINT, sigintHandler);

	if (bcm2835_init() == 0) {
			fprintf(stderr, "Not able to init the bmc2835 library\n");
			return -1;
	}


	//stepperTest();
	//tofTest();
	//IMUtest();
	//distanceTest();
	joyControl();





	//stepperTest();

	puts("Done!");

}

void tofTest(){

	bcm2835_gpio_fsel(GPIO_TOF_1, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_2, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_3, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_4, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_5, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_6, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_7, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_8, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_9, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_10, BCM2835_GPIO_FSEL_OUTP);

	//disable all sensors first
	bcm2835_gpio_clr(GPIO_TOF_1); // górny
	bcm2835_gpio_clr(GPIO_TOF_2); // górny
	bcm2835_gpio_clr(GPIO_TOF_3);
	bcm2835_gpio_clr(GPIO_TOF_4);
	bcm2835_gpio_clr(GPIO_TOF_5);
	bcm2835_gpio_clr(GPIO_TOF_6);
	bcm2835_gpio_clr(GPIO_TOF_7);
	bcm2835_gpio_clr(GPIO_TOF_8);
	bcm2835_gpio_clr(GPIO_TOF_9);
	bcm2835_gpio_clr(GPIO_TOF_10);

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

	bcm2835_gpio_set(GPIO_TOF_2);
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

	bcm2835_gpio_set(GPIO_TOF_9);
	delay(10);
	globalSensors[8] = new VL53L1X(VL53L1X::Medium,0x29);
	delay(10);
	globalSensors[8]->setAddress(0x38);
	delay(10);
	puts("Sensor nine started at: 0x38");

	bcm2835_gpio_set(GPIO_TOF_10);
	delay(10);
	globalSensors[9] = new VL53L1X(VL53L1X::Medium,0x29);
	delay(10);
	globalSensors[9]->setAddress(0x39);
	delay(10);
	puts("Sensor ten started at: 0x39");


	for(int i=0; i<10; i++){
		globalSensors[i]->startContinuous(20);
		delay(10);
	}

	while(1){
		for(int i=0; i<10; i++){
					measurement[i] = globalSensors[i]->readData(1);
					printf("%d:%5d ",i,measurement[i]);
		}
		printf("\n");
		printf("\033[H\033[J");
	}

	for(int i=0; i<10; i++){
		globalSensors[i]->disable();
		delay(10);
	}
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
	board.setSpeed(30,0);
	bcm2835_delay(10000);
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


void distanceTest(){

	bcm2835_gpio_fsel(GPIO_TOF_1, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_2, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_3, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_4, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_5, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_6, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_7, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_8, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_9, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_10, BCM2835_GPIO_FSEL_OUTP);

	//disable all sensors first
	bcm2835_gpio_clr(GPIO_TOF_1); // górny
	bcm2835_gpio_clr(GPIO_TOF_2); // górny
	bcm2835_gpio_clr(GPIO_TOF_3);
	bcm2835_gpio_clr(GPIO_TOF_4);
	bcm2835_gpio_clr(GPIO_TOF_5);
	bcm2835_gpio_clr(GPIO_TOF_6);
	bcm2835_gpio_clr(GPIO_TOF_7);
	bcm2835_gpio_clr(GPIO_TOF_8);
	bcm2835_gpio_clr(GPIO_TOF_9);
	bcm2835_gpio_clr(GPIO_TOF_10);

	long positionLeft,positionRight;
	int voltage;
	Motors board( BCM2835_SPI_CS0, GPIO_RESET_OUT);
	globalBoard = &board;
	board.setUp();
	positionLeft = board.getPositionLeft();
	positionRight = board.getPositionRight();
	voltage = board.getBatteryVoltage();

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

	bcm2835_gpio_set(GPIO_TOF_2);
	delay(10);
	globalSensors[1] = new VL53L1X(VL53L1X::Medium,0x29);
	delay(10);
	globalSensors[1]->setAddress(0x31);
	delay(10);
	puts("Sensor two started at: 0x31");

	for(int i=0; i<2; i++){
			globalSensors[i]->startContinuous(20);
			delay(10);
		}

	int target = 100;
	int speed=0;

	while(1){
		measurement[0] = globalSensors[0]->readData(1);
		measurement[1] = globalSensors[1]->readData(1);
		if(measurement[0]> target){
			speed = abs(measurement[0]-target+10);
			if(speed>300){
			speed=300;
			}
			board.setSpeed(speed,-speed);
		}
		else{
			board.setSpeed(0,0);
		}
		printf("Speed:%d\n",speed);
		printf("FullSpeed: %f\n",board.getFullSpeed());
		//board.getParam(L6470_PARAM_SPEED)
	}
	positionLeft = board.getPositionLeft();
	positionRight = board.getPositionRight();
	voltage = board.getBatteryVoltage();
	printf("Absolute position: Left:%lu		Right:%lu	Voltage:%d\n",positionLeft, positionRight,voltage);
	board.stop();


	for(int i=0; i<2; i++){
		globalSensors[i]->disable();
		delay(10);
	}

}

void joyControl(){
	int speedLeft, speedRight;
	float  ax;

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

	int f;
	int joy_fd, *axis=NULL, num_of_axis=0, num_of_buttons=0, x;
	char *button=NULL, name_of_joystick[80];
	struct js_event js;

	if( ( joy_fd = open( JOY_DEV , O_RDONLY)) == -1 )
	{
	printf( "Couldn't open joystick\n" );
	return;
	}

	ioctl( joy_fd, JSIOCGAXES, &num_of_axis );
	ioctl( joy_fd, JSIOCGBUTTONS, &num_of_buttons );
	ioctl( joy_fd, JSIOCGNAME(80), &name_of_joystick );

	axis = (int *) calloc( num_of_axis, sizeof( int ) );
	button = (char *) calloc( num_of_buttons, sizeof( char ) );

	printf("Joystick detected: %s\n\t%d axis\n\t%d buttons\n\n"
	, name_of_joystick
	, num_of_axis
	, num_of_buttons );

	fcntl( joy_fd, F_SETFL, O_NONBLOCK ); /* use non-blocking mode */

	Motors board( BCM2835_SPI_CS0, GPIO_RESET_OUT);
	globalBoard = &board;
	board.setUp();

	while( 1 )  /* infinite loop */
	{

	/* read the joystick state */
	read(joy_fd, &js, sizeof(struct js_event));

	/* see what to do with the event */
	switch (js.type & ~JS_EVENT_INIT)
	{
	case JS_EVENT_AXIS:
	axis   [ js.number ] = js.value;
	break;
	case JS_EVENT_BUTTON:
	button [ js.number ] = js.value;
	break;
	}

	//read accelerometer
	ax = SensorOne.readFloatAccelX();


	/* print the results */
	printf( "X: %6d  Y: %6d  ", axis[0], axis[1] );
	printf("  \r");

	speedLeft = axis[1]/150;
	speedRight = axis[1]/150;

	speedLeft+= axis[0]/300;
	speedRight-= axis[0]/300;

	if(speedLeft>280)speedLeft=280;
	if(speedLeft<-280)speedLeft=-280;
	if(speedRight>280)speedRight=280;
	if(speedRight<-280)speedRight=-280;

	if(ax>0.5){
		speedLeft = -1*speedLeft;
		speedRight = -1*speedRight;
	}

	board.setSpeed(speedLeft,-1*speedRight);
	//bcm2835_delay(10);


	fflush(stdout);
	}
	SensorOne.close_i2c();
	board.stop();
	close( joy_fd ); /* too bad we never get here */
	return;
}

