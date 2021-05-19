#include "nodes/CommunicationNode.hpp"

CommunicationNode::CommunicationNode(): Node("minirys_communication"){
	counter = new FrequencyCounter("communication");

	imu_data_subscriber = this->create_subscription<minirys_interfaces::msg::ImuOutput>("imu_data", 10, std::bind(&CommunicationNode::imuDataCallback, this, std::placeholders::_1));
	temperature_subscriber = this->create_subscription<sensor_msgs::msg::Temperature>("temperature", 10, std::bind(&CommunicationNode::temperatureCallback, this, std::placeholders::_1));
	battery_subscriber = this->create_subscription<sensor_msgs::msg::BatteryState>("voltage", 10, std::bind(&CommunicationNode::batteryCallback, this, std::placeholders::_1));
	tof_data_subscriber = this->create_subscription<minirys_interfaces::msg::TofOutput>("tof_data", 10, std::bind(&CommunicationNode::tofDataCallback, this, std::placeholders::_1));
	motors_controller_data_subscriber = this->create_subscription<minirys_interfaces::msg::MotorsControllerOutput>("motors_controller_data", 10, std::bind(&CommunicationNode::motorsControllerDataCallback, this, std::placeholders::_1));
	odometry_data_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("odometry_data", 10, std::bind(&CommunicationNode::odometryDataCallback, this, std::placeholders::_1));

	minirys_data_publisher = this->create_publisher<minirys_interfaces::msg::MinirysOutput>("minirys", 10);
	outputMessage = minirys_interfaces::msg::MinirysOutput();

	this->declare_parameter("period", rclcpp::ParameterValue(1));
	communication_timer = this->create_wall_timer(
	std::chrono::milliseconds(this->get_parameter("period").get_value<int>()), std::bind(&CommunicationNode::sendData, this));
	RCLCPP_INFO(this->get_logger(), "Communication node initialized.");
}

CommunicationNode::~CommunicationNode() {
	delete counter;
}

void CommunicationNode::sendData() {
	outputMessage.header.stamp = this->get_clock()->now();
	minirys_data_publisher->publish(outputMessage);
}

void CommunicationNode::motorsControllerDataCallback(const minirys_interfaces::msg::MotorsControllerOutput::SharedPtr msg) {
	// RCLCPP_INFO(this->get_logger(), "controller");
	outputMessage.motors.header = msg->header;
	outputMessage.motors.left_wheel_speed = msg->left_wheel_speed;
	outputMessage.motors.right_wheel_speed = msg->right_wheel_speed;
	outputMessage.motors.status_left = msg->status_left;
	outputMessage.motors.status_right = msg->status_right;
}

void CommunicationNode::odometryDataCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
	// RCLCPP_INFO(this->get_logger(), "odometry");
	outputMessage.odometry.header = msg->header;
	outputMessage.odometry.child_frame_id = msg->child_frame_id;
	outputMessage.odometry.pose = msg->pose;
	outputMessage.odometry.twist = msg->twist;
}

void CommunicationNode::imuDataCallback(const minirys_interfaces::msg::ImuOutput::SharedPtr msg) {
	// RCLCPP_INFO(this->get_logger(), "imu");
	outputMessage.imu.header = msg->header;
	outputMessage.imu.raw_data = msg->raw_data;
	outputMessage.imu.angle = msg->angle;
	outputMessage.imu.gyro = msg->gyro;
}

void CommunicationNode::tofDataCallback(const minirys_interfaces::msg::TofOutput::SharedPtr msg) {
	// RCLCPP_INFO(this->get_logger(), "tof");
	outputMessage.tof.header = msg->header;
	for (int i = 0; i < 6; i++) {
		outputMessage.tof.tof[i] = msg->tof[i];
	}
}

void CommunicationNode::temperatureCallback(const sensor_msgs::msg::Temperature::SharedPtr msg) {
	// RCLCPP_INFO(this->get_logger(), "temperature");
	outputMessage.temperature.header = msg->header;
	outputMessage.temperature.temperature = msg->temperature;
	outputMessage.temperature.variance = msg->variance;
}

void CommunicationNode::batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
	// RCLCPP_INFO(this->get_logger(), "voltage");
	outputMessage.battery.header = msg->header;
	outputMessage.battery.voltage = msg->voltage;
	outputMessage.battery.percentage = msg->percentage;
	outputMessage.battery.power_supply_status = msg->power_supply_status;
	outputMessage.battery.power_supply_health = msg->power_supply_health;
	outputMessage.battery.power_supply_technology = msg->power_supply_technology;
	outputMessage.battery.present = msg->present;
}