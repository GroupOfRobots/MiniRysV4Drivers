#include "nodes/MotorsControllerNode.hpp"

MotorsControllerNode::MotorsControllerNode(): Node("motors_regulator"){
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
	printMotorsStatusFromRegisters();
}

MotorsControllerNode::~MotorsControllerNode(){
	controller->disableMotors();
	delete controller;
	delete counter;
}

void MotorsControllerNode::motorsControlCallback(const minirys_interfaces::msg::MotorsControl::SharedPtr msg) {
	previousEnableBalancing = enableBalancing;
	forwardSpeed = msg->forward_speed;
	rotationSpeed = msg->rotation_speed;
	enableBalancing = msg->enable_balancing;
	printMotorStatus = msg->print_status;
}

void MotorsControllerNode::imuDataCallback(const minirys_interfaces::msg::ImuOutput::SharedPtr msg) {
	tilt = msg->angle;
	gyro = msg->angle;
}

void MotorsControllerNode::controlMotors() {
	counter->count();

	leftSpeed = 0;
	rightSpeed = 0;
	ignoreAcceleration = false;
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

	controller->setMotorSpeeds(leftSpeed, rightSpeed, ignoreAcceleration);
	leftSpeed = controller->getMotorSpeedLeft();
	rightSpeed = controller->getMotorSpeedRight();

	msg.header.stamp = this->get_clock()->now();
	msg.left_wheel_speed = leftSpeed;
	msg.right_wheel_speed = rightSpeed;
	control_publisher->publish(msg);

	if (printMotorStatus) printMotorsStatusFromRegisters();
}

void MotorsControllerNode::printMotorsSpeedConfiguration(){
	this->controller->getMotorsSpeedConfiguration(std::ref(speedConfiguration[0]), std::ref(speedConfiguration[1]), std::ref(speedConfiguration[2]), std::ref(speedConfiguration[3]));
	RCLCPP_INFO(this->get_logger(), "Motors configuration:\nMax speed:\t%f\nMin speed:\t%f\nAcceleration:\t%f\nDeceleration:\t%f\n", speedConfiguration[0], speedConfiguration[1], speedConfiguration[2], speedConfiguration[3]);
}

void MotorsControllerNode::printMotorsStatusFromRegisters(){
	this->controller->getMotorsStatusRegisters(std::ref(motorStatus0), std::ref(motorStatus1));
	RCLCPP_INFO(this->get_logger(), "Status number\t%d\n\tMOTOR: 0 - 0x%x\n\tMOTOR: 1 - 0x%x\n", statusCounter, motorStatus0, motorStatus1);
	if ((motorStatus0 & 0x7F80) != 0x7e00) {
		RCLCPP_INFO(this->get_logger(), "Something may be not right with motor 0.");
		if (~motorStatus0 & 0x4000) RCLCPP_INFO(this->get_logger(), "\tStall on bridge B.");
		if (~motorStatus0 & 0x2000) RCLCPP_INFO(this->get_logger(), "\tStall on bridge A.");
		if (~motorStatus0 & 0x1000) RCLCPP_INFO(this->get_logger(), "\tOvercurrent detected.");
		if (~motorStatus0 & 0x0800) RCLCPP_FATAL(this->get_logger(), "\tThermal shutdown detected.");
		if (~motorStatus0 & 0x0400) RCLCPP_WARN(this->get_logger(), "\tThermal warning detected.");
		if (~motorStatus0 & 0x0200) RCLCPP_INFO(this->get_logger(), "\tUndervoltage lockout or reset detected.\n\tIgnore above message if it is first status read after motor's power-up.");
		if (motorStatus0 & 0x0100) RCLCPP_INFO(this->get_logger(), "\tNon-existent command detected.");
		if (motorStatus0 & 0x0080) RCLCPP_INFO(this->get_logger(), "\tNon-performable command detected.");
	}
	if ((motorStatus1 & 0x7F80) != 0x7e00) RCLCPP_INFO(this->get_logger(), "Something may be not right with motor 1.");
		if (~motorStatus1 & 0x4000) RCLCPP_INFO(this->get_logger(), "\tStall on bridge B.");
		if (~motorStatus1 & 0x2000) RCLCPP_INFO(this->get_logger(), "\tStall on bridge A.");
		if (~motorStatus1 & 0x1000) RCLCPP_INFO(this->get_logger(), "\tOvercurrent detected.");
		if (~motorStatus1 & 0x0800) RCLCPP_FATAL(this->get_logger(), "\tThermal shutdown detected.");
		if (~motorStatus1 & 0x0400) RCLCPP_WARN(this->get_logger(), "\tThermal warning detected.");
		if (~motorStatus1 & 0x0200) RCLCPP_INFO(this->get_logger(), "\tUndervoltage lockout or reset detected.\n\tIgnore above message if it is first status read after motor's power-up.");
		if (motorStatus1 & 0x0100) RCLCPP_INFO(this->get_logger(), "\tNon-existent command detected.");
		if (motorStatus1 & 0x0080) RCLCPP_INFO(this->get_logger(), "\tNon-performable command detected.");
	statusCounter++;
}