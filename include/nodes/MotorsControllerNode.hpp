#include "rclcpp/rclcpp.hpp"
#include "../data_structures.h"
#include "../FrequencyCounter/FrequencyCounter.hpp"
#include "../MotorsController/MotorsController.hpp"
#include "minirys_interfaces/msg/motors_control.hpp"
#include "minirys_interfaces/msg/imu_output.hpp"
#include "minirys_interfaces/msg/motors_controller_output.hpp"

class MotorsControllerNode : public rclcpp::Node{
	public:
		MotorsControllerNode();
		~MotorsControllerNode();

	private:
		MotorsController *controller;
		float tilt, gyro, forwardSpeed, rotationSpeed, leftSpeed, rightSpeed;
		bool enableBalancing, previousEnableBalancing, ignoreAcceleration;
		FrequencyCounter *counter;

		int statusCounter;
		motor_status motorStatusLeft, motorStatusRight;
		float speedConfiguration[4];

		rclcpp::TimerBase::SharedPtr control_motors_timer;
		rclcpp::Subscription<minirys_interfaces::msg::MotorsControl>::SharedPtr motors_control_subscriber;
		rclcpp::Subscription<minirys_interfaces::msg::ImuOutput>::SharedPtr imu_data_subscriber;
		rclcpp::Publisher<minirys_interfaces::msg::MotorsControllerOutput>::SharedPtr control_publisher;
		minirys_interfaces::msg::MotorsControllerOutput msg;

		void controlMotors();
		void printMotorsSpeedConfiguration();
		void motorsControlCallback(const minirys_interfaces::msg::MotorsControl::SharedPtr msg);
		void imuDataCallback(const minirys_interfaces::msg::ImuOutput::SharedPtr msg);

};