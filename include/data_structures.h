#include <mutex>

#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

struct robot_control_data {
	float forwardSpeed;
	float rotationSpeed;
	bool enableBalancing;
	bool printMotorStatus;
	bool printRobotLocation;
	std::mutex robot_control_data_access;
	bool setOdometryPosition;
	float x;
	float y;
	float theta;	
	robot_control_data(): forwardSpeed(0), rotationSpeed(0), enableBalancing(false), printMotorStatus(false), printRobotLocation(false) {};
};

struct imu_data {
	float tilt;
	float gyro;
	std::mutex imu_data_access;
	imu_data(): tilt(0), gyro(0) {};
};

struct tof_data {
	float measurement[8];
	std::mutex tof_data_access;
	tof_data() {
		for(int i = 0; i < 8; i++) measurement[i] = 0.0;
	};
};

struct motor_data {
	float leftSpeed;
	float rightSpeed;
	float controller_acceleration;
	int microstep;
	std::mutex motor_data_access;
	motor_data() : leftSpeed(0), rightSpeed(0), controller_acceleration(0), microstep(0) {};
};

struct odometry_data {
	float x;
	float y;
	float theta;
	std::mutex odometry_data_access;
	odometry_data() : x(0), y(0), theta(0) {};
};

#endif