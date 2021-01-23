#include "rclcpp/rclcpp.hpp"
#include "../data_structures.h"
#include "../FrequencyCounter/FrequencyCounter.hpp"
#include "../lsm6ds3/LSM6DS3.h"
#include "../lsm6ds3/filter.h"

class ImuReaderNode : public rclcpp::Node{
	public:
		ImuReaderNode(imu_data &structure);
		~ImuReaderNode();

	private:
		float accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ, tilt;
		float gyroOffsetX, gyroOffsetY, gyroOffsetZ, angleCorrection;

		LSM6DS3 *SensorOne;// = LSM6DS3( I2C_MODE, 0x6B);
		filter *imu_filter;// = filter(0, 0, 0);
		imu_data *dataStructure = NULL; 
		FrequencyCounter counter;

		rclcpp::TimerBase::SharedPtr read_imu_data_timer;

		void read_imu_data();
};