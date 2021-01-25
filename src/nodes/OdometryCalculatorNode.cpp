#include "nodes/OdometryCalculatorNode.hpp"

OdometryCalculatorNode::OdometryCalculatorNode(motor_data& motorStructure, robot_control_data& robotControlStructure, odometry_data& odometryStructure): Node("odometry_calculator") {
	motorDataStructure = &motorStructure;
	robotControlDataStructure = &robotControlStructure;
	odometryDataStructure = &odometryStructure;

	// initialize robot position
	this->declare_parameter("initial_x", rclcpp::ParameterValue(0));
	this->declare_parameter("initial_y", rclcpp::ParameterValue(0));
	this->declare_parameter("initial_theta", rclcpp::ParameterValue(0));
	position[0] = this->get_parameter("initial_x").get_value<float>();
	position[1] = this->get_parameter("initial_y").get_value<float>();
	position[2] = this->get_parameter("initial_theta").get_value<float>();
	robotControlDataStructure->robot_control_data_access.lock();
	printRobotLocation = robotControlDataStructure->printRobotLocation;
	if (robotControlDataStructure->setOdometryPosition) setPosition(robotControlDataStructure->x, robotControlDataStructure->y, robotControlDataStructure->theta);
	robotControlDataStructure->setOdometryPosition = false;
	robotControlDataStructure->robot_control_data_access.unlock();
	if (printRobotLocation) printLocation();

	// initialize other parameters
	leftSpeed = 0;
	previousLeftSpeed = 0;
	rightSpeed = 0;
	previousRightSpeed = 0;
	acceleration = 0;
	printRobotLocation = false;

	// initialize variables for odometry calculation
	this->declare_parameter("wheel_distance", rclcpp::ParameterValue(12.7));
	wheelDistance = this->get_parameter("wheel_distance").get_value<float>()/100; // m
	this->declare_parameter("wheel_radius", rclcpp::ParameterValue(5.5));
	wheelRadius_l = this->get_parameter("wheel_radius").get_value<float>()/100; // m
	wheelRadius_r = this->get_parameter("wheel_radius").get_value<float>()/100; // m

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
	float D_l = ((leftSpeed + previousLeftSpeed)/2*accTime_l + leftSpeed*(period - accTime_l))/200*2*M_PI*wheelRadius_l; // m: step/(step/circumference)*(m/circumference)
	float D_r = ((rightSpeed + previousRightSpeed)/2*accTime_r + rightSpeed*(period - accTime_r))/200*2*M_PI*wheelRadius_r; // m: step/(step/circumference)*(m/circumference)
	float D = (D_l + D_r)/2; // m
	float changeofAngle = (D_r - D_l)/wheelDistance;
	position[2] += changeofAngle;
	cropAngle();
	position[0] = position[0] - D*sin(position[2]);
	position[1] = position[1] + D*cos(position[2]);

	odometryDataStructure->odometry_data_access.lock();
	odometryDataStructure->x = position[0];
	odometryDataStructure->y = position[1];
	odometryDataStructure->theta = position[2];
	odometryDataStructure->odometry_data_access.unlock();

	robotControlDataStructure->robot_control_data_access.lock();
	printRobotLocation = robotControlDataStructure->printRobotLocation;
	if (robotControlDataStructure->setOdometryPosition) setPosition(robotControlDataStructure->x, robotControlDataStructure->y, robotControlDataStructure->theta);
	robotControlDataStructure->setOdometryPosition = false;
	robotControlDataStructure->robot_control_data_access.unlock();

	if (printRobotLocation) printLocation();
}

void OdometryCalculatorNode::printLocation() {
	RCLCPP_INFO(this->get_logger(), "Robot location:\n\tx: %.4f\n\ty: %.4f\n\tO: %.4f", position[0], position[1], position[2]);
}

void OdometryCalculatorNode::cropAngle() {
	while (position[2] > M_PI) position[2] -= M_PI;
	while (position[2] < M_PI) position[2] += M_PI;
}

void OdometryCalculatorNode::setPosition(float ix, float iy, float itheta) {
	position[0] = ix;
	position[1] = iy;
	position[2] = itheta;
}