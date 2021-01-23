#include "rclcpp/rclcpp.hpp"
#include "../data_structures.h"
#include "../FrequencyCounter/FrequencyCounter.hpp"

class OdometryCalculatorNode : public rclcpp::Node{
	public:
		OdometryCalculatorNode(motor_data& motorStructure, robot_control_data& robotControlStructure);
	private:
		motor_data *motorDataStructure;
		robot_control_data *robotControlDataStructure;
		float period;
		float position[3];
		float velocity[3];
		float wheelDistance, wheelRadius;
		float leftSpeed, previousLeftSpeed;
		float rightSpeed, previousRightSpeed;
		float acceleration;
		bool printRobotLocation;
		FrequencyCounter counter;
		rclcpp::TimerBase::SharedPtr odometry_calculation_timer;

		void calculatePosition();
		void printLocation();
};