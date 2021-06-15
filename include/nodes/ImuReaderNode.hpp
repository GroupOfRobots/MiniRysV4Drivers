#include "rclcpp/rclcpp.hpp"
#include "../data_structures.h"
#include "../FrequencyCounter/FrequencyCounter.hpp"
#include "../lsm6ds3/LSM6DS3.h"
#include "minirys_interfaces/msg/imu_output.hpp"

class ImuReaderNode : public rclcpp::Node{
	public:
		ImuReaderNode();
		~ImuReaderNode();

	private:
		float accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ;
		float gyroOffsetX, gyroOffsetY, gyroOffsetZ, angleCorrection;

		LSM6DS3 *SensorOne;// = LSM6DS3( I2C_MODE, 0x6B);
		// filter *imu_filter;// = filter(0, 0, 0);

		float calculatedAngle;
		float previousAngles[3];
		float filteredAngle, previousFilteredAngle;
		float filteredGyro;

		FrequencyCounter *counter;

		rclcpp::TimerBase::SharedPtr read_imu_data_timer;
		rclcpp::Publisher<minirys_interfaces::msg::ImuOutput>::SharedPtr imu_publisher;
		minirys_interfaces::msg::ImuOutput msg;

		void read_imu_data();
		void filter();
};