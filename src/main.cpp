/*
 * main.cpp
 *
 *  Created on: 1 gru 2018
 *      Author: kamil
 */

#include "lsm6ds3/LSM6DS3.h"
#include "lsm6ds3/filter.h"
#include "bcm/bcm2835.h"
#include "l6470/l6470constants.h"
#include "l6470/motors.h"
#include "vl53l1x/VL53L1X.h"
#include <csignal>
#include <math.h>
#include <fstream>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
// #include "Controller/controller.h"
// For testing of old executor
#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <cstring>
#include <functional>
#include <utility>
#include <sched.h>
#include <sys/mman.h>
#include <pthread.h>
// #include "MyExecutor/MyExecutor.hpp"

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
void TMPtest();
void tofTest();
void IMUtest();
void distanceTest();
void joyControl();
void BalancingTest();
void ResponseTimeTest();
Motors *globalBoard;
VL53L1X *globalSensors[10];
uint16_t measurement[10];
std::ofstream file;

void execTest();
void f1(bool&, std::mutex&, bool&, std::string, std::chrono::milliseconds);
bool destroy = false;

void sigintHandler(int signum) {
	if (signum == SIGINT) {
		//globalBoard->Dump();
		if (file.is_open()) file.close();
		if (globalBoard != nullptr) globalBoard->stop();
		for(int i = 0; i < 10; i++)
			if (globalSensors[i] != nullptr) globalSensors[i]->disable();
		destroy = true;
		//exit(signum);
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
	//joyControl();
	//stepperTest();
	//BalancingTest();
	//ResponseTimeTest();
	//execTest();
}
/*
void execTest(){
	MyExecutor *exec = new MyExecutor(std::ref(destroy));

	bool f1_bool = false;
	std::mutex f1_mutex;
	std::string f1_name = "F10";
	std::thread t1(f1, std::ref(f1_bool), std::ref(f1_mutex), std::ref(destroy), f1_name, std::chrono::milliseconds(10));
	exec->addExec(std::ref(f1_mutex), std::ref(f1_bool), std::chrono::milliseconds(10), f1_name);

	bool f2_bool = false;
	std::mutex f2_mutex;
	std::string f2_name = "F20";
	std::thread t2(f1, std::ref(f2_bool), std::ref(f2_mutex), std::ref(destroy), f2_name, std::chrono::milliseconds(20));
	exec->addExec(std::ref(f2_mutex), std::ref(f2_bool), std::chrono::milliseconds(20), f2_name);

	bool f3_bool = false;
	std::mutex f3_mutex;
	std::string f3_name = "F40";
	std::thread t3(f1, std::ref(f3_bool), std::ref(f3_mutex), std::ref(destroy), f3_name, std::chrono::milliseconds(40));
	exec->addExec(std::ref(f3_mutex), std::ref(f3_bool), std::chrono::milliseconds(40), f3_name);

	exec->list();
	exec->spin();
	t1.join();
	t2.join();
	t3.join();
	delete exec;
	return;
}

void f1(bool& activate, std::mutex& m, bool& destroy, std::string name, std::chrono::milliseconds period){
	pthread_setname_np(pthread_self(), name.c_str());
	int numOfRuns = 0;
	float frequency = 0;
	std::chrono::time_point<std::chrono::high_resolution_clock> previous = std::chrono::high_resolution_clock::now();
	std::chrono::time_point<std::chrono::high_resolution_clock> timeNow = std::chrono::high_resolution_clock::now();

	while(!destroy){
		m.lock();
		if(activate){
			activate = false;
			m.unlock();
			numOfRuns++;
			if (numOfRuns >= 1000) {
				previous = timeNow;
				timeNow = std::chrono::high_resolution_clock::now();
				auto loopTimeSpan = std::chrono::duration_cast<std::chrono::duration<float>>(timeNow - previous);
				float loopTime = loopTimeSpan.count();
				frequency = numOfRuns/loopTime;
				printf("%s Frequency %fHz after %d runs.\n", name.c_str(), frequency, numOfRuns);
				numOfRuns = 0;
			}
		    std::this_thread::sleep_for(period/2);
		} else {
	     m.unlock();
	    }
	    std::this_thread::sleep_for(period/100);
	}
	std::cout << name << ": I'm dying.." << std::endl;
}
*/

void tofTest(){

	// Log file
	file.open("distance_log_report70cm");

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

	for(int j=0; j<1000; j++){
	//while(1){
		//for(int i=0; i<10; i++){
		//			measurement[i] = globalSensors[i]->readData(1);
		//			printf("%d:%5d ",i,measurement[i]);
		//}
				measurement[5] = globalSensors[5]->readData(1);
				printf("5:%5d ",measurement[5]);
				measurement[8] = globalSensors[8]->readData(1);
				printf("8:%5d ",measurement[8]);
		delay(20);
		file << measurement[5] <<" "<< measurement[8]<<std::endl;
		printf("\n");
		printf("\033[H\033[J");
	}

	for(int i=0; i<10; i++){
		globalSensors[i]->disable();
		delay(10);
	}

	file.close();
}

void stepperTest(){

	long positionLeft,positionRight;
	Motors board( BCM2835_SPI_CS0, GPIO_RESET_OUT);
	globalBoard = &board;
	board.setUp();
	board.resetPosition();
	positionLeft = board.getPositionLeft();
	positionRight = board.getPositionRight();
	printf("Absolute position: Left:%lu		Right:%lu \n",positionLeft, positionRight);
	board.setSpeed(-20,-20);
	bcm2835_delay(500);
	positionLeft = board.getPositionLeft();
	positionRight = board.getPositionRight();
	printf("Absolute position: Left:%lu		Right:%lu \n",positionLeft, positionRight);
	file <<"end_left:"<< positionLeft <<"end_right:"<< positionRight<<std::endl;

	board.stop();

	file.close();
}

void IMUtest(){
	LSM6DS3 SensorOne( I2C_MODE, 0x6B);

	// SensorOne.begin();
	int status = SensorOne.begin();
	if( status != 0 )
	{
		  printf("Problem number %d starting the sensor \n", status);
		  return;
	}
	else
	{
		  printf("Sensor with CS1 started, awaiting calibration.\n");
		  for (int i = 10; i>0; i--){
			if (destroy) {
				SensorOne.close_i2c();
				return;
			}
			  printf("%d\n",i);
			  delay(1000);
		  }
	}
	int i, n = 1000;

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

	// Sum of gyro readings
	float sumgx = 0, sumgy = 0, sumgz = 0;

	// Tilt
	float ty = atan2(ax,sqrt(ay*ay+az*az));

	// Filtering
	float param = 0.1;
	float freq = 10;
	filter f(ty,param,freq);
	float f_gy, f_ty;

	// Log file
	char filename [25];
	sprintf(filename,"imu_log_report_%.3f_%04.0f", param, freq);
	file.open(filename);

	for(i=0; i<n; i++){
		if (destroy) {
			SensorOne.close_i2c();
			file.close();
			return;
		}
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
		sumgx += rgx;
		sumgy += rgy;
		sumgz += rgz;
		printf(" X = %f\n",rgx);
		printf(" Y = %f",rgy);
		printf(" Z = %f\n",rgz);

		printf("\nTilt:\n");
		ty = atan2(ax,sqrt(ay*ay+az*az));
		printf(" Tilt = %f", ty);

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

		//Write to file
		file << i << " " << rgy << " " << ty << " " << f_gy << " " << f_ty << std::endl;
		delay(1000/freq);
	}
	file.close();

	printf(" Average Gyro X = %f\n", sumgx/n);
	printf(" Average Gyro Y = %f\n", sumgy/n);
	printf(" Average Gyro Z = %f\n", sumgz/n);

	SensorOne.close_i2c();
}

// void TMPtest(){ //
// 	tmp102 czujnik(0x48,"/dev/i2c-0");
// 	printf("Rys temperature: %f \n",czujnik.readTemperature());
// }

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
	int speedLeft = 0, speedRight = 0, speedLeftPast = 0, speedRightPast = 0;

	LSM6DS3 SensorOne( I2C_MODE, 0x6B);

		if( SensorOne.begin() != 0 )
		{
			  printf("Problem starting the sensor \n");
		}
		else
		{
			  printf("Sensor with CS1 started.\n");
		}
		int i, n = 10000;


	int joy_fd, *axis=NULL, num_of_axis=0, num_of_buttons=0;
	bool *button=NULL, *buttonPast=NULL;
	char name_of_joystick[80];//*button=NULL,
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
	for(int i = 0; i < num_of_axis; i++){
		axis[i] = 0;
	}
	button = (bool *) calloc( num_of_buttons, sizeof( bool ) );
	buttonPast = (bool *) calloc( num_of_buttons, sizeof( bool ) );
	for(int i = 0; i < num_of_buttons; i++){
		button[i] = false;
		buttonPast[i] = false;
	}

	printf("Joystick detected: %s\n\t%d axis\n\t%d buttons\n\n"
	, name_of_joystick
	, num_of_axis
	, num_of_buttons );

	fcntl( joy_fd, F_SETFL, O_NONBLOCK ); /* use non-blocking mode */

	Motors board( BCM2835_SPI_CS0, GPIO_RESET_OUT);
	globalBoard = &board;
	board.setUp();
	board.setSpeed(0,0);
	int filter=0, voltage = 0;

	while( 1 )  /* infinite loop */
	{
		filter++;
		voltage = board.getBatteryVoltage();
		if (filter%10000 == 0) printf("Battery:\t%d\n", voltage);

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
			if (button [ js.number ] != buttonPast[js.number]) printf("Button %d:\t%d\n", js.number, button[js.number]);
			buttonPast[js.number] = button[js.number];
			break;
		}
		speedLeft = axis[1]/80;
		speedRight = axis[1]/80;

		speedLeft+= axis[2]/200;
		speedRight-= axis[2]/200;

		if(speedLeft>400)speedLeft=400;
		if(speedLeft<-400)speedLeft=-400;
		if(speedRight>400)speedRight=400;
		if(speedRight<-400)speedRight=-400;

		//speedLeft = -1*speedLeft;
		//speedRight = -1*speedRight;
		if (speedLeft != speedLeftPast || speedRight != speedRightPast) printf("Velocity:\tLeft:%d\tRight:%d\n",speedLeft, speedRight);
		speedLeftPast = speedLeft;
		speedRightPast = speedRight;

		board.setSpeed(speedLeft,speedRight);

		fflush(stdout);
	}

	board.stop();
	close( joy_fd ); /* too bad we never get here */
	return;
}

// void BalancingTest(){

// 	float speedLeft, speedRight, oldSpeedLeft, oldSpeedRight;
// 	Controller pid;

// 	LSM6DS3 SensorOne( I2C_MODE, 0x6B);

// 	if( SensorOne.begin() != 0 )printf("Problem starting the sensor \n");
// 	else  printf("Sensor with CS1 started.\n");

// 	int joy_fd, *axis=NULL, num_of_axis=0, num_of_buttons=0;
// 	char *button=NULL, name_of_joystick[80];
// 	struct js_event js;

// 	if( ( joy_fd = open( JOY_DEV , O_RDONLY)) == -1 )
// 	{
// 	printf( "Couldn't open joystick\n" );
// 	return;
// 	}

// 	ioctl( joy_fd, JSIOCGAXES, &num_of_axis );
// 	ioctl( joy_fd, JSIOCGBUTTONS, &num_of_buttons );
// 	ioctl( joy_fd, JSIOCGNAME(80), &name_of_joystick );

// 	axis = (int *) calloc( num_of_axis, sizeof( int ) );
// 	button = (char *) calloc( num_of_buttons, sizeof( char ) );

// 	fcntl( joy_fd, F_SETFL, O_NONBLOCK ); /* use non-blocking mode */

// 	Motors board( BCM2835_SPI_CS0, GPIO_RESET_OUT);
// 	globalBoard = &board;
// 	board.setUp();

// 	// Acceleration
// 	float  ax, ay, az;
// 	ax = SensorOne.readFloatAccelX();
// 	ay = SensorOne.readFloatAccelY();
// 	az = SensorOne.readFloatAccelZ();

// 	// Gyro
// 	float rgx, rgy, rgz;
// 	const float offX = 3.597539, offY = -5.142877, offZ = -3.623744;
// 	rgx = (SensorOne.readFloatGyroX() - offX)*M_PI/180;
// 	rgy = (SensorOne.readFloatGyroY() - offY)*M_PI/180;
// 	rgz = (SensorOne.readFloatGyroZ() - offZ)*M_PI/180;

// 	// Sum of gyro readings
// 	float sumgx = 0, sumgy = 0, sumgz = 0;

// 	// Tilt
// 	float ty = atan2(ax,sqrt(ay*ay+az*az));

// 	// Filtering
// 	filter f(ty,0.5,10);
// 	float f_gy, f_ty;
// 	float angle;
// 	int steering;
// 	int throttle;

// 	while( 1 )  /* infinite loop *////////////////////////////////////////////////////
// 	{

// 		/* read the joystick state
// 		read(joy_fd, &js, sizeof(struct js_event));

// 		switch (js.type & ~JS_EVENT_INIT)
// 		{
// 		case JS_EVENT_AXIS:
// 		axis   [ js.number ] = js.value;
// 		break;
// 		case JS_EVENT_BUTTON:
// 		button [ js.number ] = js.value;
// 		break;
// 		}


// 		speedLeft = axis[1]/100;
// 		speedRight = axis[1]/100;

// 		speedLeft+= axis[3]/200;
// 		speedRight-= axis[3]/200;

// 		*/
// 		//Get all parameters
// 		ax = SensorOne.readFloatAccelX();
// 		ay = SensorOne.readFloatAccelY();
// 		az = SensorOne.readFloatAccelZ();

// 		rgx = (SensorOne.readFloatGyroX() - offX)*M_PI/180;
// 		rgy = (SensorOne.readFloatGyroY() - offY)*M_PI/180;
// 		rgz = (SensorOne.readFloatGyroZ() - offZ)*M_PI/180;

// 		ty = atan2(ax,sqrt(ay*ay+az*az));

// 		f_ty = f.getAngle(ty,rgy);
// 		f_gy = f.getGyro();


// 		angle = f_ty;
// 		delay(15);

// 		angle =angle*180/3.1415;
// 		angle -=14.5;
// 		printf(" Angle: %f\n", angle);
// 		pid.calculate_speed(angle,oldSpeedLeft,oldSpeedRight,0,0,speedLeft,speedRight);


// 		if(speedLeft>360)speedLeft=360;
// 		if(speedLeft<-360)speedLeft=-360;
// 		if(speedRight>360)speedRight=360;
// 		if(speedRight<-360)speedRight=-360;

// 		board.setSpeed((int)speedLeft,(int)speedRight);
// 		oldSpeedLeft = speedLeft;
// 		oldSpeedRight = speedRight;

// 		fflush(stdout);
// 	}
// 	SensorOne.close_i2c();
// 	board.stop();
// 	close( joy_fd ); /* too bad we never get here */
// 	return;
// }

void ResponseTimeTest(){

	// Log file
	std::ofstream file;
	file.open("response_log");
	std::chrono::high_resolution_clock::time_point timer_old = std::chrono::high_resolution_clock::now();
	std::chrono::high_resolution_clock::time_point timer_value;


	/////////////////////////////////////TOF////////////////////////////
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

	bcm2835_gpio_set(GPIO_TOF_2);
	delay(10);
	globalSensors[1] = new VL53L1X(VL53L1X::Medium,0x29);
	delay(10);
	globalSensors[1]->setAddress(0x31);
	delay(10);

	bcm2835_gpio_set(GPIO_TOF_3);
	delay(10);
	globalSensors[2] = new VL53L1X(VL53L1X::Medium,0x29);
	delay(10);
	globalSensors[2]->setAddress(0x32);
	delay(10);

	bcm2835_gpio_set(GPIO_TOF_4);
	delay(10);
	globalSensors[3] = new VL53L1X(VL53L1X::Medium,0x29);
	delay(10);
	globalSensors[3]->setAddress(0x33);
	delay(10);

	bcm2835_gpio_set(GPIO_TOF_5);
	delay(10);
	globalSensors[4] = new VL53L1X(VL53L1X::Medium,0x29);
	delay(10);
	globalSensors[4]->setAddress(0x34);
	delay(10);

	bcm2835_gpio_set(GPIO_TOF_6);
	delay(10);
	globalSensors[5] = new VL53L1X(VL53L1X::Medium,0x29);
	delay(10);
	globalSensors[5]->setAddress(0x35);
	delay(10);

	bcm2835_gpio_set(GPIO_TOF_7);
	delay(10);
	globalSensors[6] = new VL53L1X(VL53L1X::Medium,0x29);
	delay(10);
	globalSensors[6]->setAddress(0x36);
	delay(10);

	bcm2835_gpio_set(GPIO_TOF_8);
	delay(10);
	globalSensors[7] = new VL53L1X(VL53L1X::Medium,0x29);
	delay(10);
	globalSensors[7]->setAddress(0x37);
	delay(10);

	bcm2835_gpio_set(GPIO_TOF_9);
	delay(10);
	globalSensors[8] = new VL53L1X(VL53L1X::Medium,0x29);
	delay(10);
	globalSensors[8]->setAddress(0x38);
	delay(10);

	bcm2835_gpio_set(GPIO_TOF_10);
	delay(10);
	globalSensors[9] = new VL53L1X(VL53L1X::Medium,0x29);
	delay(10);
	globalSensors[9]->setAddress(0x39);
	delay(10);


	for(int i=0; i<10; i++){
		globalSensors[i]->startContinuous(20);
		delay(10);
	}


	timer_old = std::chrono::high_resolution_clock::now();
	for (int j=0; j<1000; j++){
		for(int i=0; i<10; i++){
					measurement[i] = globalSensors[i]->readData(1);
		}
	}
	timer_value = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(timer_value - timer_old);

	file << "10xTOF average time:" << std::endl;
	file << time_span.count()/1000.0 << std::endl;

	timer_old = std::chrono::high_resolution_clock::now();
	for (int j=0; j<1000; j++){
		measurement[0] = globalSensors[0]->readData(1);
	}
	timer_value = std::chrono::high_resolution_clock::now();
	time_span = std::chrono::duration_cast<std::chrono::duration<double>>(timer_value - timer_old);

	file << "1xTOF average time:" << std::endl;
	file << time_span.count()/1000.0 << std::endl;

	for(int i=0; i<10; i++){
		globalSensors[i]->disable();
		delay(10);
	}

	puts("TOF done");

	/////////////////////////////////////IMU///////////////////////////////

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
	float f_gy, f_ty,temperature;


	timer_old = std::chrono::high_resolution_clock::now();
	for(i=0; i<n; i++){
		//Get all parameters
		ax = SensorOne.readFloatAccelX();
		ay = SensorOne.readFloatAccelY();
		az = SensorOne.readFloatAccelZ();
		rgx = (SensorOne.readFloatGyroX() - offX)*M_PI/180;
		rgy = (SensorOne.readFloatGyroY() - offY)*M_PI/180;
		rgz = (SensorOne.readFloatGyroZ() - offZ)*M_PI/180;
		temperature = SensorOne.readTempC();
	}
	timer_value = std::chrono::high_resolution_clock::now();
	time_span = std::chrono::duration_cast<std::chrono::duration<double>>(timer_value - timer_old);

	file << "IMU_all average time:" << std::endl;
	file << time_span.count()/1000.0 << std::endl;

	SensorOne.close_i2c();

	puts("IMU done");
	/////////////////////////////////////TMP102///////////////////////////////
	// tmp102 czujnik(0x48,"/dev/i2c-0");

	// timer_old = std::chrono::high_resolution_clock::now();
	// for (int j=0; j<1000; j++){
	// 	temperature = czujnik.readTemperature();
	// }
	// timer_value = std::chrono::high_resolution_clock::now();
	// time_span = std::chrono::duration_cast<std::chrono::duration<double>>(timer_value - timer_old);

	// file << "1xTEMP average time:" << std::endl;
	// file << time_span.count()/1000.0 << std::endl;

	// puts("TMP done");

	/////////////////////////////////////Steppers///////////////////////////////

	long positionLeft,positionRight;
	int voltage;
	Motors board( BCM2835_SPI_CS0, GPIO_RESET_OUT);
	globalBoard = &board;
	board.setUp();

	timer_old = std::chrono::high_resolution_clock::now();
	for (int j=0; j<1000; j++){
		board.setSpeed(400,400);
		positionLeft = board.getPositionLeft();
		positionRight = board.getPositionRight();
	}
	timer_value = std::chrono::high_resolution_clock::now();
	time_span = std::chrono::duration_cast<std::chrono::duration<double>>(timer_value - timer_old);

	file << "2xSTEPPER + position average time:" << std::endl;
	file << time_span.count()/1000.0 << std::endl;

	puts("Steppers done");

	timer_old = std::chrono::high_resolution_clock::now();
	for (int j=0; j<1000; j++){
		voltage = board.getBatteryVoltage();
	}
	timer_value = std::chrono::high_resolution_clock::now();
	time_span = std::chrono::duration_cast<std::chrono::duration<double>>(timer_value - timer_old);

	file << "1xBatteryV average time:" << std::endl;
	file << time_span.count()/1000.0 << std::endl;

	puts("temp done");

	board.stop();


	file.close();
}


