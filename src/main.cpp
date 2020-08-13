#include <chrono>
#include <memory>
#include <sched.h>
#include <sys/mman.h>
#include <mutex>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "bcm/bcm2835.h"
#include "lsm6ds3/LSM6DS3.h"
#include "lsm6ds3/filter.h"
#include "vl53l1x/VL53L1X.h"
#include "MotorsController/MotorsController.hpp"
#include "FrequencyCounter/FrequencyCounter.hpp"
#include "vl53l1x_stm/vl53l1_api.h"
#include "vl53l1x_stm/vl53l1_platform.h"


#define JOY_DEV "/dev/input/js0"
#define GPIO_TOF_1 	RPI_V2_GPIO_P1_12
// #define GPIO_TOF_2 	RPI_V2_GPIO_P1_16
#define GPIO_TOF_3 	RPI_V2_GPIO_P1_18
#define GPIO_TOF_4 	RPI_V2_GPIO_P1_29
#define GPIO_TOF_5	RPI_V2_GPIO_P1_32
#define GPIO_TOF_6 	RPI_V2_GPIO_P1_31
#define GPIO_TOF_7 	RPI_V2_GPIO_P1_33
#define GPIO_TOF_8 	RPI_V2_GPIO_P1_35
#define GPIO_TOF_9 	RPI_V2_GPIO_P1_36
#define NUM_OF_TOF 8
// #define NUM_OF_TOF 9

using std::placeholders::_1;
using namespace std::chrono_literals;

struct joycon_data {
	float forwardSpeed;
	float rotationSpeed;
	bool enableBalancing;
	bool printStatus;
	std::mutex joycon_data_access;
	joycon_data(): forwardSpeed(0), rotationSpeed(0), enableBalancing(false), printStatus(false) {};
};

struct imu_data {
	float tilt;
	float gyro;
	std::mutex imu_data_access;
	imu_data(): tilt(0), gyro(0) {};
};

struct tof_data {
	float measurement[NUM_OF_TOF];
	std::mutex tof_data_access;
	tof_data() {
		for(int i = 0; i < NUM_OF_TOF; i++) measurement[i] = 0.0;
	};
};

bool endProcess = false;

void sigintHandler(int signum) {
	if (signum == SIGINT) {
		endProcess = true;
	}
}

void setRTPriority() {
	struct sched_param schedulerParams;
	schedulerParams.sched_priority = sched_get_priority_max(SCHED_FIFO)-1;
	std::cout << "[MAIN] Setting RT scheduling, priority " << schedulerParams.sched_priority << std::endl;
	if (sched_setscheduler(0, SCHED_FIFO, &schedulerParams) == -1) {
		std::cout << "[MAIN] WARNING: Setting RT scheduling failed: " << std::strerror(errno) << std::endl;
		return;
	}

	if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
		std::cout << "[MAIN] WARNING: Failed to lock memory: " << std::strerror(errno) << std::endl;
	}
}

class JoyconReceiver : public rclcpp::Node{
	public:
		JoyconReceiver(joycon_data& structure): Node("joycon_receiver") {
			dataStructure = &structure;
			//initialize joycon
			if( ( joy_fd = open( JOY_DEV , O_RDONLY)) == -1 )
			{
				RCLCPP_ERROR(this->get_logger(), "Couldn't open joystick" );
				endProcess = true;
				return;
			}

			ioctl( joy_fd, JSIOCGAXES, &num_of_axis );
			ioctl( joy_fd, JSIOCGBUTTONS, &num_of_buttons );
			ioctl( joy_fd, JSIOCGNAME(80), &name_of_joystick );

			axis = (int *) calloc( num_of_axis, sizeof( int ) );
			axisPast = (int *) calloc( num_of_axis, sizeof( int ) );
			for(int i = 0; i < num_of_axis; i++){
				axis[i] = 0;
				axisPast[i] = 0;
			}
			button = (bool *) calloc( num_of_buttons, sizeof( bool ) );
			buttonPast = (bool *) calloc( num_of_buttons, sizeof( bool ) );
			for(int i = 0; i < num_of_buttons; i++){
				button[i] = false;
				buttonPast[i] = false;
			}

			RCLCPP_INFO(this->get_logger(),"Joystick detected: %s\n\t%d axis\n\t%d buttons\n\n"
			, name_of_joystick
			, num_of_axis
			, num_of_buttons );

			fcntl( joy_fd, F_SETFL, O_NONBLOCK ); /* use non-blocking mode */

			// Get axis parameters
			this->declare_parameter("forwardAxis", rclcpp::ParameterValue(1));
			forwardAxis = this->get_parameter("forwardAxis").get_value<int>();
			this->declare_parameter("rotationAxis", rclcpp::ParameterValue(2));
			rotationAxis = this->get_parameter("rotationAxis").get_value<int>();
			this->declare_parameter("forwardSpeedFactor", rclcpp::ParameterValue(80));
			forwardSpeedFactor = this->get_parameter("forwardSpeedFactor").get_value<int>();
			this->declare_parameter("rotationSpeedFactor", rclcpp::ParameterValue(200));
			rotationSpeedFactor = this->get_parameter("rotationSpeedFactor").get_value<int>();
			this->declare_parameter("forwardAxisInverted", rclcpp::ParameterValue(true));
			forwardAxisInverted = this->get_parameter("forwardAxisInverted").get_value<bool>();
			this->declare_parameter("rotationAxisInverted", rclcpp::ParameterValue(false));
			rotationAxisInverted = this->get_parameter("rotationAxisInverted").get_value<bool>();

			// Get button parameters
			this->declare_parameter("standUpButton", rclcpp::ParameterValue(1));
			standUpButton = this->get_parameter("standUpButton").get_value<int>();
			this->declare_parameter("layDownButton", rclcpp::ParameterValue(3));
			layDownButton = this->get_parameter("layDownButton").get_value<int>();
			this->declare_parameter("printStatusButton", rclcpp::ParameterValue(2));
			printStatusButton = this->get_parameter("printStatusButton").get_value<int>();

			this->declare_parameter("period", rclcpp::ParameterValue(10));
			get_joycon_state_timer = this->create_wall_timer(
			std::chrono::milliseconds(this->get_parameter("period").get_value<int>()), std::bind(&JoyconReceiver::get_joycon_state, this));
			RCLCPP_INFO(this->get_logger(), "Joycon receiver initialized.");
		}

		~JoyconReceiver(){
			close( joy_fd );
		}

	private:
		int joy_fd, *axis = NULL, *axisPast = NULL, num_of_axis = 0, num_of_buttons = 0;
		bool *button = NULL, *buttonPast = NULL;
		char name_of_joystick[80];
		struct js_event js;

		int forwardAxis, rotationAxis, forwardSpeedFactor, rotationSpeedFactor;
		bool forwardAxisInverted, rotationAxisInverted;

		int standUpButton, layDownButton, printStatusButton;

		joycon_data *dataStructure = NULL; 
		FrequencyCounter counter;

		rclcpp::TimerBase::SharedPtr get_joycon_state_timer;
		void get_joycon_state() {
			counter.count();
			/* read the joystick state */
			/* see what to do with the event */
			while(read(joy_fd, &js, sizeof(struct js_event)) == sizeof(struct js_event)){
				switch (js.type & ~JS_EVENT_INIT)
				{
					case JS_EVENT_AXIS:
						axis   [ js.number ] = js.value;
						// if (axis [ js.number ] != axisPast[js.number]) RCLCPP_INFO(this->get_logger(),"Axis %d:\t%d", js.number, axis[js.number]);
						axisPast[js.number] = axis[js.number];
						break;
					case JS_EVENT_BUTTON:
						button [ js.number ] = js.value;
						if (button [ js.number ] != buttonPast[js.number]) RCLCPP_INFO(this->get_logger(),"Button %d:\t%d", js.number, button[js.number]);
						buttonPast[js.number] = button[js.number];
						break;
				}
			}

			dataStructure->joycon_data_access.lock();

			if(!forwardAxisInverted){
				dataStructure->forwardSpeed = axis[forwardAxis]/forwardSpeedFactor;
			} else {
				dataStructure->forwardSpeed = -axis[forwardAxis]/forwardSpeedFactor;
			}

			if(!rotationAxisInverted){
				dataStructure->rotationSpeed = axis[rotationAxis]/rotationSpeedFactor;
			} else {
				dataStructure->rotationSpeed = -axis[rotationAxis]/rotationSpeedFactor;
			}

			if (button[standUpButton] == 1) dataStructure->enableBalancing = true;
			if (button[layDownButton] == 1) dataStructure->enableBalancing = false;
			dataStructure->printStatus = button[printStatusButton] == 1 ? true : false;

			dataStructure->joycon_data_access.unlock();
		}
};

class ImuReader : public rclcpp::Node{
	public:
		ImuReader(imu_data &structure): Node("imu_reader") {
			SensorOne = new LSM6DS3( I2C_MODE, 0x6B);
			int status, i;
			for (i = 0; i < 10; i++){
				status = SensorOne->begin();
				if( status != 0 )
				{
					RCLCPP_ERROR(this->get_logger(), "Problem number %d starting the sensor, retrying...", status);
				} else break;
			}

			if (i == 10) {
				RCLCPP_ERROR(this->get_logger(), "Unable to start imu sensor...");
				endProcess = true;
			} else RCLCPP_INFO(this->get_logger(), "IMU sensor started, awaiting calibration.");

			for (i = 10; i>0; i--){
				if (endProcess) break;
				RCLCPP_INFO(this->get_logger(), "%d seconds to calibration...",i);
				delay(1000);
			}

			dataStructure = &structure;

			accelerationX = SensorOne->readFloatAccelX();
			accelerationY = SensorOne->readFloatAccelY();
			accelerationZ = SensorOne->readFloatAccelZ();

			this->declare_parameter("gyroOffsetX", rclcpp::ParameterValue(0.0));
			this->declare_parameter("gyroOffsetY", rclcpp::ParameterValue(0.0));
			this->declare_parameter("gyroOffsetZ", rclcpp::ParameterValue(0.0));
			gyroOffsetX = this->get_parameter("gyroOffsetX").get_value<float>();
			gyroOffsetY = this->get_parameter("gyroOffsetY").get_value<float>();
			gyroOffsetZ = this->get_parameter("gyroOffsetZ").get_value<float>();
			gyroX = (SensorOne->readFloatGyroX() - gyroOffsetX)*M_PI/180;
			gyroY = (SensorOne->readFloatGyroX() - gyroOffsetY)*M_PI/180;
			gyroZ = (SensorOne->readFloatGyroX() - gyroOffsetZ)*M_PI/180;

			this->declare_parameter("angleCorrection", rclcpp::ParameterValue(0.0));
			angleCorrection = this->get_parameter("angleCorrection").get_value<float>();
			tilt = atan2(accelerationX, sqrt(accelerationY*accelerationY + accelerationZ*accelerationZ)) - angleCorrection;
			this->declare_parameter("filterFactor", rclcpp::ParameterValue(0.05));
			this->declare_parameter("period", rclcpp::ParameterValue(10));
			imu_filter = new filter(tilt, this->get_parameter("filterFactor").get_value<float>(), 1000/this->get_parameter("period").get_value<int>());

			read_imu_data_timer = this->create_wall_timer(
			std::chrono::milliseconds(this->get_parameter("period").get_value<int>()), std::bind(&ImuReader::read_imu_data, this));
			RCLCPP_INFO(this->get_logger(), "IMU reader initialized.");
		}

		~ImuReader(){
			SensorOne->close_i2c();
			delete SensorOne;
		}

	private:
		float accelerationX = 0, accelerationY = 0, accelerationZ = 0, gyroX = 0, gyroY = 0, gyroZ = 0, tilt = 0;
		float gyroOffsetX, gyroOffsetY, gyroOffsetZ, angleCorrection;

		LSM6DS3 *SensorOne;// = LSM6DS3( I2C_MODE, 0x6B);
		filter *imu_filter;// = filter(0, 0, 0);
		imu_data *dataStructure = NULL; 
		FrequencyCounter counter;

		rclcpp::TimerBase::SharedPtr read_imu_data_timer;
		void read_imu_data() {
			counter.count();
			accelerationX = SensorOne->readFloatAccelX();
			accelerationY = SensorOne->readFloatAccelY();
			accelerationZ = SensorOne->readFloatAccelZ();
			gyroX = (SensorOne->readFloatGyroX() - gyroOffsetX)*M_PI/180;
			gyroY = (SensorOne->readFloatGyroY() - gyroOffsetY)*M_PI/180;
			gyroZ = (SensorOne->readFloatGyroZ() - gyroOffsetZ)*M_PI/180;
			tilt = atan2(accelerationX, sqrt(accelerationY*accelerationY + accelerationZ*accelerationZ)) - angleCorrection;

			dataStructure->imu_data_access.lock();
			dataStructure->tilt = imu_filter->getAngle(tilt, gyroY);
			// RCLCPP_INFO(this->get_logger(), "%f\t%f\n", tilt, dataStructure->tilt);
			dataStructure->gyro = imu_filter->getGyro();
			dataStructure->imu_data_access.unlock();
			// RCLCPP_INFO(this->get_logger(), "Debug");
		}
};

class TOFReader : public rclcpp::Node{
	public:
		TOFReader(tof_data &structure): Node("tof_reader"){

			dataStructure = &structure;

			bcm2835_gpio_fsel(GPIO_TOF_1, BCM2835_GPIO_FSEL_OUTP);
			// bcm2835_gpio_fsel(GPIO_TOF_2, BCM2835_GPIO_FSEL_OUTP);
			bcm2835_gpio_fsel(GPIO_TOF_3, BCM2835_GPIO_FSEL_OUTP);
			bcm2835_gpio_fsel(GPIO_TOF_4, BCM2835_GPIO_FSEL_OUTP);
			bcm2835_gpio_fsel(GPIO_TOF_5, BCM2835_GPIO_FSEL_OUTP);
			bcm2835_gpio_fsel(GPIO_TOF_6, BCM2835_GPIO_FSEL_OUTP);
			bcm2835_gpio_fsel(GPIO_TOF_7, BCM2835_GPIO_FSEL_OUTP);
			bcm2835_gpio_fsel(GPIO_TOF_8, BCM2835_GPIO_FSEL_OUTP);
			bcm2835_gpio_fsel(GPIO_TOF_9, BCM2835_GPIO_FSEL_OUTP);

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

			bcm2835_i2c_begin(); //begin I2C
			// bcm2835_i2c_set_baudrate(40000);

			//enable sensor one and change address
			bcm2835_gpio_set(GPIO_TOF_1);
			tofSensors[0] = new VL53L1X(VL53L1X::Medium,0x29);
			tofSensors[0]->setAddress(0x30);
			RCLCPP_INFO(this->get_logger(), "TOF sensor one started at: 0x30");

			// bcm2835_gpio_set(GPIO_TOF_2);
			bcm2835_gpio_set(GPIO_TOF_9);
			tofSensors[1] = new VL53L1X(VL53L1X::Medium,0x29);
			tofSensors[1]->setAddress(0x31);
			RCLCPP_INFO(this->get_logger(), "TOF sensor two started at: 0x31");

			bcm2835_gpio_set(GPIO_TOF_3);
			tofSensors[2] = new VL53L1X(VL53L1X::Medium,0x29);
			tofSensors[2]->setAddress(0x32);
			RCLCPP_INFO(this->get_logger(), "TOF sensor three started at: 0x32");

			bcm2835_gpio_set(GPIO_TOF_4);
			tofSensors[3] = new VL53L1X(VL53L1X::Medium,0x29);
			tofSensors[3]->setAddress(0x33);
			RCLCPP_INFO(this->get_logger(), "TOF sensor four started at: 0x33");

			bcm2835_gpio_set(GPIO_TOF_5);
			tofSensors[4] = new VL53L1X(VL53L1X::Medium,0x29);
			tofSensors[4]->setAddress(0x34);
			RCLCPP_INFO(this->get_logger(), "TOF sensor five started at: 0x34");

			bcm2835_gpio_set(GPIO_TOF_6);
			tofSensors[5] = new VL53L1X(VL53L1X::Medium,0x29);
			tofSensors[5]->setAddress(0x35);
			RCLCPP_INFO(this->get_logger(), "TOF sensor six started at: 0x35");

			bcm2835_gpio_set(GPIO_TOF_7);
			tofSensors[6] = new VL53L1X(VL53L1X::Medium,0x29);
			tofSensors[6]->setAddress(0x36);
			RCLCPP_INFO(this->get_logger(), "TOF sensor seven started at: 0x36");

			bcm2835_gpio_set(GPIO_TOF_8);
			tofSensors[7] = new VL53L1X(VL53L1X::Medium,0x29);
			tofSensors[7]->setAddress(0x37);
			RCLCPP_INFO(this->get_logger(), "TOF sensor eight started at: 0x37");

			// bcm2835_gpio_set(GPIO_TOF_9);
			// tofSensors[8] = new VL53L1X(VL53L1X::Medium,0x29);
			// tofSensors[8]->setAddress(0x38);
			// RCLCPP_INFO(this->get_logger(), "TOF sensor nine started at: 0x38");

			// c = 0;

			this->declare_parameter("period", rclcpp::ParameterValue(100));
			for(int i = 0; i < NUM_OF_TOF; i++){
				tofSensors[i]->startContinuous(this->get_parameter("period").get_value<int>());
			}
			read_tof_data_timer = this->create_wall_timer(
			std::chrono::milliseconds(this->get_parameter("period").get_value<int>()), std::bind(&TOFReader::read_tof_data, this));
		}

		~TOFReader() {
			for(int i = 0; i < NUM_OF_TOF; i++){
				tofSensors[i]->disable();
				delete tofSensors[i];
			}
		}
	private:
		VL53L1X *tofSensors[NUM_OF_TOF];
		tof_data *dataStructure = NULL;
		FrequencyCounter counter;
		// int c;

		rclcpp::TimerBase::SharedPtr read_tof_data_timer;
		void read_tof_data() {
			counter.count();
			// c++;
			// RCLCPP_INFO(this->get_logger(), "Init debug text %d", c);
			dataStructure->tof_data_access.lock();
			for(int i = 0; i < NUM_OF_TOF; i++){
				dataStructure->measurement[i] = tofSensors[i]->readData(1);
			}
			// RCLCPP_INFO(this->get_logger(), "End debug text %d", c);
			dataStructure->tof_data_access.unlock();
		}
};

class MotorsRegulator : public rclcpp::Node{
	public:
		MotorsRegulator(imu_data& imuStructure, joycon_data& joyconStructure): Node("motors_regulator"){
			controller = new MotorsController();
			// RCLCPP_INFO(this->get_logger(), "Motors class object initialized.");
			// printMotorStatus();
			controller->enableMotors();
			imuDataStructure = &imuStructure;
			joyconDataStructure = &joyconStructure;
			controller->setBalancing(false);

			this->declare_parameter("enableSpeedPID", rclcpp::ParameterValue(false));
			controller->setPIDSpeedRegulatorEnabled(this->get_parameter("enableSpeedPID").get_value<bool>());
		    this->declare_parameter("pidSpeedKp", rclcpp::ParameterValue(0.0));
		    this->declare_parameter("pidSpeedInvTi", rclcpp::ParameterValue(0.0));
		    this->declare_parameter("pidSpeedTd", rclcpp::ParameterValue(0.0));
		    this->declare_parameter("pidAngleKp", rclcpp::ParameterValue(0.0));
		    this->declare_parameter("pidAngleInvTi", rclcpp::ParameterValue(0.0));
		    this->declare_parameter("pidAngleTd", rclcpp::ParameterValue(0.0));
		    controller->setPIDParameters(
			    this->get_parameter("pidSpeedKp").get_value<float>(),
			    this->get_parameter("pidSpeedInvTi").get_value<float>(),
			    this->get_parameter("pidSpeedTd").get_value<float>(),
			    this->get_parameter("pidAngleKp").get_value<float>(),
			    this->get_parameter("pidAngleInvTi").get_value<float>(),
			    this->get_parameter("pidAngleTd").get_value<float>());

			this->declare_parameter("microstep", rclcpp::ParameterValue(64));
			controller->setMicrostep(this->get_parameter("microstep").get_value<int>());
			// RCLCPP_INFO(this->get_logger(), "Microstep set.");
			// printMotorStatus();
			this->declare_parameter("maxSpeed", rclcpp::ParameterValue(400.0));
			controller->setMaxSpeed(this->get_parameter("maxSpeed").get_value<float>());
			// RCLCPP_INFO(this->get_logger(), "Max speed set.");
			// printMotorStatus();
			this->declare_parameter("maxAcceleration", rclcpp::ParameterValue(100.0));
			controller->setMaxAcceleration(this->get_parameter("maxAcceleration").get_value<float>());
			this->declare_parameter("invertLeftMotor", rclcpp::ParameterValue(true));
			this->declare_parameter("invertRightMotor", rclcpp::ParameterValue(true));
			controller->setInvertSpeed(this->get_parameter("invertLeftMotor").get_value<bool>(), this->get_parameter("invertRightMotor").get_value<bool>());
			this->declare_parameter("period", rclcpp::ParameterValue(10));
			control_motors_timer = this->create_wall_timer(
			std::chrono::milliseconds(this->get_parameter("period").get_value<int>()), std::bind(&MotorsRegulator::controlMotors, this));
			RCLCPP_INFO(this->get_logger(), "Motor controller initialized.");

			statusCounter = 0;
			printMotorsSpeedConfiguration();
			printMotorsStatusFromRegisters();
		}

		~MotorsRegulator(){
			controller->disableMotors();
			delete controller;
		}
	private:
		MotorsController *controller;
		joycon_data *joyconDataStructure;
		imu_data *imuDataStructure;
		float tilt, gyro, forwardSpeed, rotationSpeed, leftSpeed, rightSpeed;
		bool enableBalancing, previousEnableBalancing, ignoreAcceleration;
		FrequencyCounter counter;

		bool printStatus;
		int statusCounter;
		long motorStatus0, motorStatus1;
		float speedConfiguration[4];

		rclcpp::TimerBase::SharedPtr control_motors_timer;
		void controlMotors() {
			counter.count();
			imuDataStructure->imu_data_access.lock();
			tilt = imuDataStructure->tilt;
			gyro = imuDataStructure->gyro;
			imuDataStructure->imu_data_access.unlock();
			// RCLCPP_INFO(this->get_logger(), "%3.4f\t%3.4f", tilt, gyro);

			previousEnableBalancing = enableBalancing;
			joyconDataStructure->joycon_data_access.lock();
			forwardSpeed = joyconDataStructure->forwardSpeed;
			rotationSpeed = joyconDataStructure->rotationSpeed;
			enableBalancing = joyconDataStructure->enableBalancing;
			printStatus = joyconDataStructure->printStatus;
			joyconDataStructure->joycon_data_access.unlock();

			leftSpeed = 0;
			rightSpeed = 0;
			ignoreAcceleration = false;
			controller->setPIDSpeedRegulatorEnabled(this->get_parameter("enableSpeedPID").get_value<bool>());
		    controller->setPIDParameters(
			    this->get_parameter("pidSpeedKp").get_value<float>(),
			    this->get_parameter("pidSpeedInvTi").get_value<float>(),
			    this->get_parameter("pidSpeedTd").get_value<float>(),
			    this->get_parameter("pidAngleKp").get_value<float>(),
			    this->get_parameter("pidAngleInvTi").get_value<float>(),
			    this->get_parameter("pidAngleTd").get_value<float>());

			// RCLCPP_INFO(this->get_logger(), "\t%s\t%s", enableBalancing ? "true" : "false", previousEnableBalancing ? "true" : "false");
			if (enableBalancing && !previousEnableBalancing) {
				controller->resetStandUp();
				controller->zeroPIDRegulator();
			}

			if (!enableBalancing && controller->getBalancing()) {
				controller->setBalancing(false);
				controller->calculateSpeeds(tilt, gyro, forwardSpeed, rotationSpeed, std::ref(leftSpeed), std::ref(rightSpeed), (float)(this->get_parameter("period").get_value<int>())/1000);
			}else if (enableBalancing && !controller->getBalancing()) {
				controller->standUp(tilt, std::ref(leftSpeed), std::ref(rightSpeed));
				// if (controller->getBalancing()) controller->calculateSpeeds(tilt, gyro, forwardSpeed, rotationSpeed, std::ref(leftSpeed), std::ref(rightSpeed), (float)(this->get_parameter("period").get_value<int>())/1000);
			} else controller->calculateSpeeds(tilt, gyro, forwardSpeed, rotationSpeed, std::ref(leftSpeed), std::ref(rightSpeed), (float)(this->get_parameter("period").get_value<int>())/1000);
			// RCLCPP_INFO(this->get_logger(), "%3.4f\t%3.4f\t%3.4f\t%3.4f", forwardSpeed, rotationSpeed, leftSpeed, rightSpeed);

			controller->setMotorSpeeds(leftSpeed, rightSpeed, ignoreAcceleration);
			leftSpeed = controller->getMotorSpeedLeft();
			rightSpeed = controller->getMotorSpeedRight();
			// RCLCPP_INFO(this->get_logger(), "%3.4f\t%3.4f\t%3.4f\t%3.4f", forwardSpeed, rotationSpeed, leftSpeed, rightSpeed);
			// RCLCPP_INFO(this->get_logger(), "\t%1.4f\t%3.4f\t%3.4f", tilt, leftSpeed, rightSpeed);
			// printMotorsStatusFromRegisters();

			if (printStatus) printMotorsStatusFromRegisters();
		}

		void printMotorsSpeedConfiguration(){
			this->controller->getMotorsSpeedConfiguration(std::ref(speedConfiguration[0]), std::ref(speedConfiguration[1]), std::ref(speedConfiguration[2]), std::ref(speedConfiguration[3]));
			RCLCPP_INFO(this->get_logger(), "Motors configuration:\nMax speed:\t%f\nMin speed:\t%f\nAcceleration:\t%f\nDeceleration:\t%f\n", speedConfiguration[0], speedConfiguration[1], speedConfiguration[2], speedConfiguration[3]);
		}

		void printMotorsStatusFromRegisters(){
			this->controller->getMotorsStatusRegisters(std::ref(motorStatus0), std::ref(motorStatus1));
			RCLCPP_INFO(this->get_logger(), "Status number\t%d\n\tMOTOR: 0 - 0x%x\n\tMOTOR: 1 - 0x%x\n", statusCounter, motorStatus0, motorStatus1);
			if ((motorStatus0 & 0x7F80) != 0x7e00) {
				RCLCPP_INFO(this->get_logger(), "Something may be not right with motor 0.");
				if (~motorStatus0 & 0x4000) RCLCPP_INFO(this->get_logger(), "\tStall on bridge B.");
				if (~motorStatus0 & 0x2000) RCLCPP_INFO(this->get_logger(), "\tStall on bridge A.");
				if (~motorStatus0 & 0x1000) RCLCPP_INFO(this->get_logger(), "\tOvercurrent detected.");
				if (~motorStatus0 & 0x0800) RCLCPP_INFO(this->get_logger(), "\tThermal shutdown detected.");
				if (~motorStatus0 & 0x0400) RCLCPP_INFO(this->get_logger(), "\tThermal warning detected.");
				if (~motorStatus0 & 0x0200) RCLCPP_INFO(this->get_logger(), "\tUndervoltage lockout or reset detected.\n\tIgnore above message if it is first status read after motor's power-up.");
				if (motorStatus0 & 0x0100) RCLCPP_INFO(this->get_logger(), "\tNon-existent command detected.");
				if (motorStatus0 & 0x0080) RCLCPP_INFO(this->get_logger(), "\tNon-performable command detected.");
			}
			if ((motorStatus1 & 0x7F80) != 0x7e00) RCLCPP_INFO(this->get_logger(), "Something may be not right with motor 1.");
				if (~motorStatus1 & 0x4000) RCLCPP_INFO(this->get_logger(), "\tStall on bridge B.");
				if (~motorStatus1 & 0x2000) RCLCPP_INFO(this->get_logger(), "\tStall on bridge A.");
				if (~motorStatus1 & 0x1000) RCLCPP_INFO(this->get_logger(), "\tOvercurrent detected.");
				if (~motorStatus1 & 0x0800) RCLCPP_INFO(this->get_logger(), "\tThermal shutdown detected.");
				if (~motorStatus1 & 0x0400) RCLCPP_INFO(this->get_logger(), "\tThermal warning detected.");
				if (~motorStatus1 & 0x0200) RCLCPP_INFO(this->get_logger(), "\tUndervoltage lockout or reset detected.\n\tIgnore above message if it is first status read after motor's power-up.");
				if (motorStatus1 & 0x0100) RCLCPP_INFO(this->get_logger(), "\tNon-existent command detected.");
				if (motorStatus1 & 0x0080) RCLCPP_INFO(this->get_logger(), "\tNon-performable command detected.");
			statusCounter++;
		}

};
// class Remote : public rclcpp::Node{}

class TOFSTM : public rclcpp::Node{

	public:
	
		TOFSTM(tof_data &structure): Node("tof_reader_stm"){
			dataStructure = &structure;

			// setting GPIO direction
			for (int i = 0; i < NUM_OF_TOF; i++) bcm2835_gpio_fsel(sensorsPins[i], BCM2835_GPIO_FSEL_OUTP);

			//disable all sensors first
			for (int i = 0; i < NUM_OF_TOF; i++) bcm2835_gpio_clr(sensorsPins[i]);	

			this->declare_parameter("sensorsUsed", rclcpp::ParameterValue(6));
			sensorsUsed = this->get_parameter("sensorsUsed").get_value<int>();

			//sensors' and sata structures
			for (int s = 0; s < sensorsUsed; ++s) {
				sensor[s] = new VL53L1_Dev_t();
				sensor[s]->I2cDevAddr = 0x29;
				data[s] = new VL53L1_RangingMeasurementData_t();
			}

			RCLCPP_INFO(this->get_logger(), "Starting TOF sensors and setting i2c addresses.");
			for (int s = 0; s < sensorsUsed; ++s)
			{
				bcm2835_gpio_set(sensorsPins[s]);
				bcm2835_delay(100);
				VL53L1_SetDeviceAddress(sensor[s], sensorsAddress[s]*2);
				sensor[s]->I2cDevAddr = sensorsAddress[s];
				bcm2835_delay(100);
			}

			RCLCPP_INFO(this->get_logger(), "Device booted.");
			for (int s = 0; s < sensorsUsed; ++s)
			{
				VL53L1_WaitDeviceBooted(sensor[s]);
			}
			bcm2835_delay(100);

			RCLCPP_INFO(this->get_logger(), "Data init.");
			for (int s = 0; s < sensorsUsed; ++s)
			{
				VL53L1_DataInit(sensor[s]);
			}
			bcm2835_delay(100);

			// static initialization
			RCLCPP_INFO(this->get_logger(), "Static init.");
			for (int s = 0; s < sensorsUsed; ++s)
			{
				VL53L1_StaticInit(sensor[s]);
			}
			bcm2835_delay(100);

			// measurement timing budget initialization
			this->declare_parameter("mtb", rclcpp::ParameterValue(20000));
			mtb = this->get_parameter("mtb").get_value<int>();
			RCLCPP_INFO(this->get_logger(), "Setting measurement timing budget.");
			for (int s = 0; s < sensorsUsed; ++s)
			{
				VL53L1_SetMeasurementTimingBudgetMicroSeconds(sensor[s], mtb);
			}
			bcm2835_delay(100);

			// inter measurement period initialization
			this->declare_parameter("imp", rclcpp::ParameterValue(24));
			imp = this->get_parameter("imp").get_value<int>();
			RCLCPP_INFO(this->get_logger(), "Setting inter measurement period.");
			for (int s = 0; s < sensorsUsed; ++s)
			{
				VL53L1_SetInterMeasurementPeriodMilliSeconds(sensor[s], imp);
			}
			bcm2835_delay(100);

			// start sensors' measurements
			RCLCPP_INFO(this->get_logger(), "Starting measurements.");
			for (int s = 0; s < sensorsUsed; ++s)
			{
				VL53L1_StartMeasurement(sensor[s]);
			}
			bcm2835_delay(100);

			this->declare_parameter("period", rclcpp::ParameterValue(100));
			read_tof_data_timer = this->create_wall_timer(
			std::chrono::milliseconds(this->get_parameter("period").get_value<int>()), std::bind(&TOFSTM::read_tof_data, this));
		}

		~TOFSTM(){
			// stop measurement process
			for (int s = 0; s < sensorsUsed; ++s)
			{
				VL53L1_StopMeasurement(sensor[s]);
				delete sensor[s];
				delete data[s];
			}
		}

	private:
		VL53L1_Dev_t *sensor[NUM_OF_TOF];
		VL53L1_RangingMeasurementData_t *data[NUM_OF_TOF];
		uint8_t dataReady[NUM_OF_TOF];
		uint8_t ready;
		tof_data *dataStructure = NULL;
		FrequencyCounter counter;
		int sensorsUsed, mtb, imp;
		const uint8_t sensorsPins[NUM_OF_TOF] = {GPIO_TOF_1, GPIO_TOF_3, GPIO_TOF_4, GPIO_TOF_5, GPIO_TOF_6, GPIO_TOF_7, GPIO_TOF_8, GPIO_TOF_9};
		const uint8_t sensorsAddress[NUM_OF_TOF] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37};
		rclcpp::TimerBase::SharedPtr read_tof_data_timer;

		void read_tof_data(){
			counter.count();

			do {
				ready = 0;
				for (int s = 0; s < sensorsUsed; ++s) {
					VL53L1_GetMeasurementDataReady(sensor[s], &dataReady[s]);
					ready += dataReady[s];
			// 		printf("%d ", dataReady[s]);
				}
			// 	printf("\n");
			} while(ready != 6);

			for (int s = 0; s < sensorsUsed; ++s)
			{
				VL53L1_WaitMeasurementDataReady(sensor[s]);
				VL53L1_GetRangingMeasurementData(sensor[s], data[s]);
				VL53L1_ClearInterruptAndStartMeasurement(sensor[s]);
			}
			// RCLCPP_INFO(this->get_logger(), "%d, %d, %d, %d, %d, %d", data[0]->RangeMilliMeter, data[1]->RangeMilliMeter, data[2]->RangeMilliMeter, data[3]->RangeMilliMeter, data[4]->RangeMilliMeter, data[5]->RangeMilliMeter);
			dataStructure->tof_data_access.lock();
			for (int s = 0; s < sensorsUsed; ++s)
			{
				dataStructure->measurement[s] = (float)data[s]->RangeMilliMeter;
			}
			dataStructure->tof_data_access.unlock();
		}
	
};

int main(int argc, char * argv[]) {
	setbuf(stdout, nullptr);
	setRTPriority();
	if (bcm2835_init() == 0) {
		fprintf(stderr, "Not able to init the bmc2835 library\n");
		return -1;
	}
	rclcpp::init(argc, argv);
	// signal(SIGINT, sigintHandler);
	rclcpp::executors::MultiThreadedExecutor executor;

	joycon_data joycon_data_structure;
	auto JoyconReceiverNode = std::make_shared<JoyconReceiver>(std::ref(joycon_data_structure));
	executor.add_node(JoyconReceiverNode);

	imu_data imu_data_structure;
	auto ImuReaderNode = std::make_shared<ImuReader>(std::ref(imu_data_structure));
	executor.add_node(ImuReaderNode);

	tof_data tof_data_structure;
	// auto TOFReaderNode = std::make_shared<TOFReader>(std::ref(tof_data_structure));
	// executor.add_node(TOFReaderNode);
	// auto TOFReaderNodeSTM = std::make_shared<TOFSTM>(std::ref(tof_data_structure));
	// executor.add_node(TOFReaderNodeSTM);

	auto MotorsRegulatorNode = std::make_shared<MotorsRegulator>(std::ref(imu_data_structure), std::ref(joycon_data_structure));
	executor.add_node(MotorsRegulatorNode);

	// while(!endProcess) executor.spin_some();
	executor.spin();
	rclcpp::shutdown();
	return 0;
}