#include "nodes/OdometryCalculatorNode.hpp"
#include <cmath>
#include <algorithm>

OdometryCalculatorNode::OdometryCalculatorNode(): Node("odometry_calculator") {
	counter = new FrequencyCounter("odometry");

	// initialize robot position
	this->declare_parameter("initial_x", rclcpp::ParameterValue(0.0));
	this->declare_parameter("initial_y", rclcpp::ParameterValue(0.0));
	this->declare_parameter("initial_theta", rclcpp::ParameterValue(0.0));
	x = this->get_parameter("initial_x").get_value<float>();
	y = this->get_parameter("initial_y").get_value<float>();
	angle = this->get_parameter("initial_theta").get_value<float>();

	// initialize other parameters
	speed_l = 0;
	setSpeed_l = 0;
	previousSetSpeed_l = 0;
	speed_r = 0;
	setSpeed_r = 0;
	previousSetSpeed_r = 0;

	// initialize variables for odometry calculation
	this->declare_parameter("motors_acceleration", rclcpp::ParameterValue(1500.0));
	acceleration = this->get_parameter("motors_acceleration").get_value<float>();
	this->declare_parameter("wheel_distance", rclcpp::ParameterValue(12.7));
	this->declare_parameter("e_b", rclcpp::ParameterValue(1.0));
	wheelDistance = this->get_parameter("e_b").get_value<float>()*this->get_parameter("wheel_distance").get_value<float>(); // m
	this->declare_parameter("wheel_radius", rclcpp::ParameterValue(5.5));
	this->declare_parameter("e_d", rclcpp::ParameterValue(1.0));
	wheelRadius_l = this->get_parameter("wheel_radius").get_value<float>()*2/(this->get_parameter("e_d").get_value<float>()+1); // m
	wheelRadius_r = this->get_parameter("wheel_radius").get_value<float>()*2/(1/this->get_parameter("e_d").get_value<float>()+1); // m
	currentTime = (double)this->get_clock()->now().nanoseconds()/1000000000;
	lastMessageTime = currentTime;
	accelerationTime_l = 0;
	accelerationTime_r = 0;
	distance_l = 0;
	distance_r = 0;
	phaseDuration = 0;

	// RCLCPP_INFO(this->get_logger(), "%f\t%f\t%f", wheelDistance, wheelRadius_l, wheelRadius_r);

	this->declare_parameter("period", rclcpp::ParameterValue(1));
	period = (double)this->get_parameter("period").get_value<int>()/1000; // s
	motors_controller_subscriber = this->create_subscription<minirys_interfaces::msg::MotorsControllerOutput>("motors_controller_data", 10, std::bind(&OdometryCalculatorNode::receiveCurrentSetSpeeds, this, std::placeholders::_1));
	odometry_publisher = this->create_publisher<nav_msgs::msg::Odometry>("odometry_data", 10);
	msg = nav_msgs::msg::Odometry();
	set_position_service = this->create_service<minirys_interfaces::srv::SetOdometryPosition>("set_odometry_position", std::bind(&OdometryCalculatorNode::setOdometryPosition, this, std::placeholders::_1, std::placeholders::_2));
	odometry_calculation_timer = this->create_wall_timer(
	std::chrono::milliseconds(this->get_parameter("period").get_value<int>()), std::bind(&OdometryCalculatorNode::updatePosition, this));
	RCLCPP_INFO(this->get_logger(), "Odometry calculator initialized.");
}

OdometryCalculatorNode::~OdometryCalculatorNode() {
	delete counter;
}

void OdometryCalculatorNode::updatePosition() {
	counter->count();
	currentTime = (double)this->get_clock()->now().nanoseconds()/1000000000;
	distance_l = 0;
	distance_r = 0;

	phaseDuration = period - (currentTime - lastMessageTime); //time from previous run to new message, if negative there was no new message since last run
	if (phaseDuration > 0) { //new message from controller was obtained, calculate position at that time
		accelerationTime_l = 0;
		accelerationTime_r = 0;
		if (previousSetSpeed_l != speed_l) { //acceleration left
			accelerationTime_l = std::min(abs(previousSetSpeed_l - speed_l)/acceleration, phaseDuration);
			distance_l += (speed_l*accelerationTime_l + (previousSetSpeed_l > speed_l ? 1 : -1)*acceleration*pow(accelerationTime_l, 2)/2)*STEPS_TO_RAD*wheelRadius_l;
			speed_l += accelerationTime_l*acceleration*(previousSetSpeed_l > speed_l ? 1 : -1);
		}
		if (previousSetSpeed_r != speed_r) { // acceleration right
			accelerationTime_r = std::min(abs(previousSetSpeed_r - speed_r)/acceleration, phaseDuration);
			distance_r += (speed_r*accelerationTime_r + (previousSetSpeed_r > speed_r ? 1 : -1)*acceleration*pow(accelerationTime_r, 2)/2)*STEPS_TO_RAD*wheelRadius_r;
			speed_r += accelerationTime_r*acceleration*(previousSetSpeed_r > speed_r ? 1 : -1);
		}
		// driving straight forward
		distance_l += speed_l*(phaseDuration - accelerationTime_l)*STEPS_TO_RAD*wheelRadius_l;
		distance_r += speed_r*(phaseDuration - accelerationTime_r)*STEPS_TO_RAD*wheelRadius_r;
	}

	phaseDuration = period - (phaseDuration > 0 ? phaseDuration : 0); // time from last message to now or this functions period, whichevers lower
	accelerationTime_l = 0;
	accelerationTime_r = 0;
	if (setSpeed_l != speed_l) { //acceleration left
		accelerationTime_l = std::min(abs(setSpeed_l - speed_l)/acceleration, phaseDuration);
		distance_l += (speed_l*accelerationTime_l + (setSpeed_l > speed_l ? 1 : -1)*acceleration*pow(accelerationTime_l, 2)/2)*STEPS_TO_RAD*wheelRadius_l;
		speed_l += accelerationTime_l*acceleration*(setSpeed_l > speed_l ? 1 : -1);
	}
	if (setSpeed_r != speed_r) { // acceleration right
		accelerationTime_r = std::min(abs(setSpeed_r - speed_r)/acceleration, phaseDuration);
		distance_r += (speed_r*accelerationTime_r + (setSpeed_r > speed_r ? 1 : -1)*acceleration*pow(accelerationTime_r, 2)/2)*STEPS_TO_RAD*wheelRadius_r;
		speed_r += accelerationTime_r*acceleration*(setSpeed_r > speed_r ? 1 : -1);
	}
	// driving straight forward
	distance_l += speed_l*(phaseDuration - accelerationTime_l)*STEPS_TO_RAD*wheelRadius_l;
	distance_r += speed_r*(phaseDuration - accelerationTime_r)*STEPS_TO_RAD*wheelRadius_r;

	angle += (distance_r - distance_l)/wheelDistance;
	cropAngle();
	x = x + (distance_l+distance_r)/2*cos(angle);
	y = y + (distance_l+distance_r)/2*sin(angle);

	msg.header.stamp = this->get_clock()->now();
	msg.pose.pose.position.x = x;
	msg.pose.pose.position.y = y;
	msg.pose.pose.orientation.w = cos(angle/2);
	msg.pose.pose.orientation.x = 0;
	msg.pose.pose.orientation.y = 0;
	msg.pose.pose.orientation.z = sin(angle/2);
	odometry_publisher->publish(msg);

	// printLocation();
}

void OdometryCalculatorNode::receiveCurrentSetSpeeds(const minirys_interfaces::msg::MotorsControllerOutput::SharedPtr msg) {
	previousSetSpeed_l = setSpeed_l;
	previousSetSpeed_r = setSpeed_r;
	lastMessageTime = (double)msg->header.stamp.sec + (double)msg->header.stamp.nanosec/1000000000;
	if (setSpeed_l != msg->left_wheel_speed){
		setSpeed_l = msg->left_wheel_speed;
	}
	if (setSpeed_r != msg->right_wheel_speed){
		setSpeed_r = msg->right_wheel_speed;
	}
}

void OdometryCalculatorNode::printLocation() {
	RCLCPP_INFO(this->get_logger(), "Robot location:\n\tx: %.4f\n\ty: %.4f\n\tO: %.4f", x, y, angle);
}

void OdometryCalculatorNode::cropAngle() {
	if (angle > M_PI) angle -= 2*M_PI;
	if (angle < -M_PI) angle += 2*M_PI;
}

void OdometryCalculatorNode::setOdometryPosition(const std::shared_ptr<minirys_interfaces::srv::SetOdometryPosition::Request> request,
			std::shared_ptr<minirys_interfaces::srv::SetOdometryPosition::Response> response){
	x = request.get()->pose.position.x;
	y = request.get()->pose.position.y;
	angle = atan2(2*(request.get()->pose.orientation.w*request.get()->pose.orientation.z+request.get()->pose.orientation.x*request.get()->pose.orientation.y),
		1-2*(pow(request.get()->pose.orientation.y, 2)+pow(request.get()->pose.orientation.z, 2)));
}

void OdometryCalculatorNode::setPosition(double ix, double iy, double itheta) {
	x = ix;
	y = iy;
	angle = itheta;
}