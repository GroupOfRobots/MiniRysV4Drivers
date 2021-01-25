#include "rclcpp/rclcpp.hpp"
#include "../data_structures.h"
#include "../FrequencyCounter/FrequencyCounter.hpp"

class OdometryCalculatorNode : public rclcpp::Node{
	public:
		OdometryCalculatorNode(motor_data&, robot_control_data&, odometry_data&);
	private:
		motor_data *motorDataStructure;
		robot_control_data *robotControlDataStructure;
		odometry_data *odometryDataStructure;
		float period;
		float position[3];
		float wheelDistance, wheelRadius_l, wheelRadius_r;
		float leftSpeed, previousLeftSpeed;
		float rightSpeed, previousRightSpeed;
		float acceleration;
		bool printRobotLocation;
		FrequencyCounter counter;
		rclcpp::TimerBase::SharedPtr odometry_calculation_timer;

		void calculatePosition();
		void printLocation();
		void cropAngle();
		void setPosition(float, float, float);
};