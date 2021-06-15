#include "nodes/MotorsControllerNode.hpp"

MotorsControllerNode::MotorsControllerNode(): Node("motors_controller"){
	counter = new FrequencyCounter("motors");
	controller = new MotorsController();
	controller->enableMotors();
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

	motors_control_subscriber = this->create_subscription<minirys_interfaces::msg::MotorsControl>("motors_control", 10, std::bind(&MotorsControllerNode::motorsControlCallback, this, std::placeholders::_1));
	imu_data_subscriber = this->create_subscription<minirys_interfaces::msg::ImuOutput>("imu_data", 10, std::bind(&MotorsControllerNode::imuDataCallback, this, std::placeholders::_1));
	control_publisher = this->create_publisher<minirys_interfaces::msg::MotorsControllerOutput>("motors_controller_data", 10);
	msg = minirys_interfaces::msg::MotorsControllerOutput();

	this->declare_parameter("period", rclcpp::ParameterValue(10));
	control_motors_timer = this->create_wall_timer(
	std::chrono::milliseconds(this->get_parameter("period").get_value<int>()), std::bind(&MotorsControllerNode::controlMotors, this));
	RCLCPP_INFO(this->get_logger(), "Motor controller initialized.");

	statusCounter = 0;
	printMotorsSpeedConfiguration();

	forwardSpeed = 0;
	rotationSpeed = 0;
	enableBalancing = false;
	previousEnableBalancing = false;
}

MotorsControllerNode::~MotorsControllerNode(){
	controller->disableMotors();
	delete controller;
	delete counter;
}

void MotorsControllerNode::motorsControlCallback(const minirys_interfaces::msg::MotorsControl::SharedPtr msg) {
	forwardSpeed = msg->forward_speed;
	rotationSpeed = msg->rotation_speed;
	enableBalancing = msg->enable_balancing;
}

void MotorsControllerNode::imuDataCallback(const minirys_interfaces::msg::ImuOutput::SharedPtr msg) {
	tilt = msg->angle;
	gyro = msg->angle;
}

void MotorsControllerNode::controlMotors() {
	counter->count();

	leftSpeed = 0;
	rightSpeed = 0;
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

	controller->setMotorSpeeds(leftSpeed, rightSpeed, false);
	leftSpeed = controller->getMotorSpeedLeft();
	rightSpeed = controller->getMotorSpeedRight();

	// build message
	msg.header.stamp = this->get_clock()->now();
	msg.left_wheel_speed = leftSpeed;
	msg.right_wheel_speed = rightSpeed;
	// this->controller->getMotorsStatusRegisters(std::ref(motorStatusLeft), std::ref(motorStatusRight));
	motorStatusLeft = controller->getMotorStatusLeft();
	motorStatusRight = controller->getMotorStatusRight();
	// left motor status
	msg.status_left.stall_bridge_b = motorStatusLeft.status[0];
	msg.status_left.stall_bridge_a = motorStatusLeft.status[1];
	msg.status_left.overcurrent = motorStatusLeft.status[2];
	msg.status_left.thermal_shutdown = motorStatusLeft.status[3];
	msg.status_left.thermal_warning = motorStatusLeft.status[4];
	msg.status_left.undervoltage = motorStatusLeft.status[5];
	msg.status_left.motor_stopped = motorStatusLeft.status[6];
	msg.status_left.motor_accelerating = motorStatusLeft.status[7];
	msg.status_left.motor_decelerating = motorStatusLeft.status[8];
	msg.status_left.motor_running = motorStatusLeft.status[9];
	msg.status_left.motor_direction = motorStatusLeft.status[10];
	msg.status_left.busy = motorStatusLeft.status[11];
	msg.status_left.high_impedance = motorStatusLeft.status[12];
	// right motor status
	msg.status_right.stall_bridge_b = motorStatusRight.status[0];
	msg.status_right.stall_bridge_a = motorStatusRight.status[1];
	msg.status_right.overcurrent = motorStatusRight.status[2];
	msg.status_right.thermal_shutdown = motorStatusRight.status[3];
	msg.status_right.thermal_warning = motorStatusRight.status[4];
	msg.status_right.undervoltage = motorStatusRight.status[5];
	msg.status_right.motor_stopped = motorStatusRight.status[6];
	msg.status_right.motor_accelerating = motorStatusRight.status[7];
	msg.status_right.motor_decelerating = motorStatusRight.status[8];
	msg.status_right.motor_running = motorStatusRight.status[9];
	msg.status_right.motor_direction = motorStatusRight.status[10];
	msg.status_right.busy = motorStatusRight.status[11];
	msg.status_right.high_impedance = motorStatusRight.status[12];
	// send message
	control_publisher->publish(msg);

	// printMotorsStatusFromRegisters();
	previousEnableBalancing = enableBalancing;
}

void MotorsControllerNode::printMotorsSpeedConfiguration(){
	this->controller->getMotorsSpeedConfiguration(std::ref(speedConfiguration[0]), std::ref(speedConfiguration[1]), std::ref(speedConfiguration[2]), std::ref(speedConfiguration[3]));
	RCLCPP_INFO(this->get_logger(), "Motors configuration:\nMax speed:\t%f\nMin speed:\t%f\nAcceleration:\t%f\nDeceleration:\t%f\n", speedConfiguration[0], speedConfiguration[1], speedConfiguration[2], speedConfiguration[3]);
}