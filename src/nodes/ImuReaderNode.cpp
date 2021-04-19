#include "nodes/ImuReaderNode.hpp"

ImuReaderNode::ImuReaderNode(): Node("imu_reader") {
	counter = new FrequencyCounter("imu");
	SensorOne = new LSM6DS3( I2C_MODE, 0x6B);
	int status, i;
	for (i = 0; i < 10; i++){
		status = SensorOne->begin();
		if( status != 0 )
		{
			RCLCPP_ERROR(this->get_logger(), "Problem number %d starting the sensor, retrying...", status);
		} else break;
	}

	if (i == 10) {
		RCLCPP_ERROR(this->get_logger(), "Unable to start imu sensor...");
		// endProcess = true;
	} else RCLCPP_INFO(this->get_logger(), "IMU sensor started, awaiting calibration.");

	for (i = 10; i>0; i--){
		// if (endProcess) break;
		RCLCPP_INFO(this->get_logger(), "%d seconds to calibration...",i);
		delay(1000);
	}

	accelerationX = SensorOne->readFloatAccelX();
	accelerationY = SensorOne->readFloatAccelY();
	accelerationZ = SensorOne->readFloatAccelZ();

	this->declare_parameter("gyroOffsetX", rclcpp::ParameterValue(0.0));
	this->declare_parameter("gyroOffsetY", rclcpp::ParameterValue(0.0));
	this->declare_parameter("gyroOffsetZ", rclcpp::ParameterValue(0.0));
	gyroOffsetX = this->get_parameter("gyroOffsetX").get_value<float>();
	gyroOffsetY = this->get_parameter("gyroOffsetY").get_value<float>();
	gyroOffsetZ = this->get_parameter("gyroOffsetZ").get_value<float>();
	gyroX = (SensorOne->readFloatGyroX() - gyroOffsetX)*M_PI/180;
	gyroY = (SensorOne->readFloatGyroX() - gyroOffsetY)*M_PI/180;
	gyroZ = (SensorOne->readFloatGyroX() - gyroOffsetZ)*M_PI/180;

	this->declare_parameter("angleCorrection", rclcpp::ParameterValue(0.0));
	angleCorrection = this->get_parameter("angleCorrection").get_value<float>();
	calculatedAngle = atan2(accelerationX, sqrt(accelerationY*accelerationY + accelerationZ*accelerationZ)) - angleCorrection;
	previousAngles[0] = calculatedAngle;
	previousAngles[1] = calculatedAngle;
	previousAngles[2] = calculatedAngle;
	filteredAngle = calculatedAngle;
	previousFilteredAngle = calculatedAngle;
	filteredGyro = gyroY;
	this->declare_parameter("filterFactor", rclcpp::ParameterValue(0.05));
	this->declare_parameter("period", rclcpp::ParameterValue(10));

	imu_publisher = this->create_publisher<minirys_interfaces::msg::ImuOutput>("imu_data", 10);
	msg = minirys_interfaces::msg::ImuOutput();

	read_imu_data_timer = this->create_wall_timer(
	std::chrono::milliseconds(this->get_parameter("period").get_value<int>()), std::bind(&ImuReaderNode::read_imu_data, this));
	RCLCPP_INFO(this->get_logger(), "IMU reader initialized.");
}

ImuReaderNode::~ImuReaderNode(){
	SensorOne->close_i2c();
	delete SensorOne;
	delete counter;
}

void ImuReaderNode::read_imu_data() {
	counter->count();
	accelerationX = SensorOne->readFloatAccelX();
	accelerationY = SensorOne->readFloatAccelY();
	accelerationZ = SensorOne->readFloatAccelZ();
	gyroX = (SensorOne->readFloatGyroX() - gyroOffsetX)*M_PI/180;
	gyroY = (SensorOne->readFloatGyroY() - gyroOffsetY)*M_PI/180;
	gyroZ = (SensorOne->readFloatGyroZ() - gyroOffsetZ)*M_PI/180;
	calculatedAngle = atan2(accelerationX, sqrt(accelerationY*accelerationY + accelerationZ*accelerationZ)) - angleCorrection;

	filter();

	msg.header.stamp = this->get_clock()->now();
	msg.angle = filteredAngle;
	msg.gyro = filteredGyro;
	msg.raw_data.linear_acceleration.x = accelerationX;
	msg.raw_data.linear_acceleration.y = accelerationY;
	msg.raw_data.linear_acceleration.z = accelerationZ;
	msg.raw_data.angular_velocity.x = gyroX;
	msg.raw_data.angular_velocity.y = gyroY;
	msg.raw_data.angular_velocity.z = gyroZ;

	imu_publisher->publish(msg);
}

void ImuReaderNode::filter() {
	previousFilteredAngle = filteredAngle;
	filteredAngle = (filteredAngle + gyroY*this->get_parameter("period").get_value<int>()/1000)*(1-this->get_parameter("filterFactor").get_value<float>())+this->get_parameter("filterFactor").get_value<float>()*(calculatedAngle+previousAngles[0]+previousAngles[1]+previousAngles[2])/4;
	filteredGyro = (filteredAngle - previousFilteredAngle)*1000/this->get_parameter("period").get_value<int>();
	previousAngles[2] = previousAngles[1];
	previousAngles[1] = previousAngles[0];
	previousAngles[0] = calculatedAngle;
}