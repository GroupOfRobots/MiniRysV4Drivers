#include "rclcpp/rclcpp.hpp"
#include "../data_structures.h"
#include "../FrequencyCounter/FrequencyCounter.hpp"
#include "../MotorsController/MotorsController.hpp"

class MotorsControllerNode : public rclcpp::Node{
	public:
		MotorsControllerNode(imu_data& imuStructure, robot_control_data& robotControlStructure, motor_data& motorStructure);
		~MotorsControllerNode();

	private:
		MotorsController *controller;
		robot_control_data *robotControlDataStructure;
		imu_data *imuDataStructure;
		motor_data *motorDataStructure;
		float tilt, gyro, forwardSpeed, rotationSpeed, leftSpeed, rightSpeed;
		bool enableBalancing, previousEnableBalancing, ignoreAcceleration;
		FrequencyCounter counter;

		bool printMotorStatus;
		int statusCounter;
		long motorStatus0, motorStatus1;
		float speedConfiguration[4];

		rclcpp::TimerBase::SharedPtr control_motors_timer;

		void controlMotors();
		void printMotorsSpeedConfiguration();
		void printMotorsStatusFromRegisters();

};