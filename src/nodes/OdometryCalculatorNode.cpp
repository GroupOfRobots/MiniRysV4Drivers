#include "nodes/OdometryCalculatorNode.hpp"

OdometryCalculatorNode::OdometryCalculatorNode(motor_data& motorStructure, robot_control_data& robotControlStructure): Node("odometry_calculator") {
	motorDataStructure = &motorStructure;
	robotControlDataStructure = &robotControlStructure;

	for(int i = 0; i < 3; i++) {
		position[i] = 0.0;
		velocity[i] = 0.0;
	}
	leftSpeed = 0;
	previousLeftSpeed = 0;
	rightSpeed = 0;
	previousRightSpeed = 0;
	acceleration = 0;
	printRobotLocation = false;

	this->declare_parameter("wheel_distance", rclcpp::ParameterValue(12.7));
	wheelDistance = this->get_parameter("wheel_distance").get_value<float>()/100; // m
	this->declare_parameter("wheel_radius", rclcpp::ParameterValue(5.5));
	wheelRadius = this->get_parameter("wheel_radius").get_value<float>()/100; // m

	this->declare_parameter("period", rclcpp::ParameterValue(10));
	period = (float)this->get_parameter("period").get_value<int>()/1000; // s
	odometry_calculation_timer = this->create_wall_timer(
	std::chrono::milliseconds(this->get_parameter("period").get_value<int>()), std::bind(&OdometryCalculatorNode::calculatePosition, this));
	RCLCPP_INFO(this->get_logger(), "Odometry calculator initialized.");
}

void OdometryCalculatorNode::calculatePosition() {
	counter.count();
	previousLeftSpeed = leftSpeed;
	previousRightSpeed = rightSpeed;
	motorDataStructure->motor_data_access.lock();
	leftSpeed = motorDataStructure->leftSpeed; // step/s
	rightSpeed = motorDataStructure->rightSpeed; // step/s
	acceleration = motorDataStructure->controller_acceleration*100; // step/s2: step/s/period = step/s/10ms = step/0.01s2 = 100*step/s2
	motorDataStructure->motor_data_access.unlock();

	float accTime_l = abs(leftSpeed - previousLeftSpeed)/acceleration; // s
	float accTime_r = abs(rightSpeed - previousRightSpeed)/acceleration; // s
	float D_l = ((leftSpeed + previousLeftSpeed)/2*accTime_l + leftSpeed*(period - accTime_l))/200*2*M_PI*wheelRadius; // m: step/(step/circumference)*(m/circumference)
	float D_r = ((rightSpeed + previousRightSpeed)/2*accTime_r + rightSpeed*(period - accTime_r))/200*2*M_PI*wheelRadius; // m: step/(step/circumference)*(m/circumference)
	float D = (D_l + D_r)/2; // m
	float theta = (D_r - D_l)/wheelDistance;
	position[2] += theta;
	position[0] = position[0] + D*cos(position[2]);
	position[1] = position[1] + D*sin(position[2]);

	robotControlDataStructure->robot_control_data_access.lock();
	printRobotLocation = robotControlDataStructure->printRobotLocation;
	robotControlDataStructure->robot_control_data_access.unlock();

	if (printRobotLocation) printLocation();
}

void OdometryCalculatorNode::printLocation() {
	RCLCPP_INFO(this->get_logger(), "Robot location:\n\tx: %.4f\n\ty: %.4f\n\tO: %.4f", position[0], position[1], position[2]);
}