#include "nodes/CommunicationNode.hpp"

CommunicationNode::CommunicationNode(): Node("minirys_communication"){
	counter = new FrequencyCounter("communication");

	minirys_data_publisher = this->create_publisher<minirys_interfaces::msg::MinirysOutput>("minirys_output", 10);
	outputMessage = minirys_interfaces::msg::MinirysOutput();
	motors_control_publisher = this->create_publisher<minirys_interfaces::msg::MotorsControl>("motors_control", 10);
	motorsMessage = minirys_interfaces::msg::MotorsControl();

	imu_data_subscriber = this->create_subscription<minirys_interfaces::msg::ImuOutput>("imu_data", 10, std::bind(&CommunicationNode::imuDataCallback, this, std::placeholders::_1));
	temperature_subscriber = this->create_subscription<sensor_msgs::msg::Temperature>("temperature", 10, std::bind(&CommunicationNode::temperatureCallback, this, std::placeholders::_1));
	battery_subscriber = this->create_subscription<sensor_msgs::msg::BatteryState>("voltage", 10, std::bind(&CommunicationNode::batteryCallback, this, std::placeholders::_1));
	tof_data_subscriber = this->create_subscription<minirys_interfaces::msg::TofOutput>("tof_data", 10, std::bind(&CommunicationNode::tofDataCallback, this, std::placeholders::_1));
	motors_controller_data_subscriber = this->create_subscription<minirys_interfaces::msg::MotorsControllerOutput>("motors_controller_data", 10, std::bind(&CommunicationNode::motorsControllerDataCallback, this, std::placeholders::_1));
	odometry_data_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("odometry_data", 10, std::bind(&CommunicationNode::odometryDataCallback, this, std::placeholders::_1));
	minirys_subscriber = this->create_subscription<minirys_interfaces::msg::MinirysInput>("minirys_input", 10, std::bind(&CommunicationNode::minirysCallback, this, std::placeholders::_1));

	set_odometry_position_client = this->create_client<minirys_interfaces::srv::SetOdometryPosition>("set_odometry_position");
	set_odometry_position_request = std::make_shared<minirys_interfaces::srv::SetOdometryPosition::Request>();

	this->declare_parameter("period", rclcpp::ParameterValue(1));
	communication_timer = this->create_wall_timer(
	std::chrono::milliseconds(this->get_parameter("period").get_value<int>()), std::bind(&CommunicationNode::sendData, this));
	RCLCPP_INFO(this->get_logger(), "Communication node initialized.");
}

CommunicationNode::~CommunicationNode() {
	delete counter;
}

void CommunicationNode::minirysCallback(const minirys_interfaces::msg::MinirysInput::SharedPtr msg) {
	// RCLCPP_INFO(this->get_logger(), "input");
	outputMessage.emergency_stop = outputMessage.emergency_stop || msg->emergency_shutdown;

	if (outputMessage.emergency_stop) {
		motorsMessage.forward_speed = 0;
		motorsMessage.rotation_speed = 0;
		motorsMessage.enable_balancing = false;
	} else {
		motorsMessage.forward_speed = msg->motor_control.forward_speed;
		motorsMessage.rotation_speed = msg->motor_control.rotation_speed;
		motorsMessage.enable_balancing = msg->motor_control.enable_balancing;
	}
	motors_control_publisher->publish(motorsMessage);

	if (msg->set_new_odometry_position && !outputMessage.emergency_stop) {
		set_odometry_position_request->pose = msg->new_odometry_position;
		set_odometry_position_client->async_send_request(set_odometry_position_request);
		// RCLCPP_INFO(this->get_logger(), "New odometry position set.");
	}
}

void CommunicationNode::sendData() {
	counter->count();
	// RCLCPP_INFO(this->get_logger(), "Data sent.");
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
	for (int i = 0; i < 8; i++) {
		outputMessage.tof.sensor[i] = msg->sensor[i];
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

	if (outputMessage.battery.power_supply_health == sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_DEAD) outputMessage.emergency_stop = true;
}