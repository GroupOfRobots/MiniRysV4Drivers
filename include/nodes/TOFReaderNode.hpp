#include "rclcpp/rclcpp.hpp"
#include "../FrequencyCounter/FrequencyCounter.hpp"
#include "../vl53l1x/VL53L1X.h"
#include "../bcm/bcm2835.h"
#include "minirys_interfaces/msg/tof_output.hpp"

#define GPIO_TOF_1 	RPI_V2_GPIO_P1_12
// #define GPIO_TOF_2 	RPI_V2_GPIO_P1_16
#define GPIO_TOF_3 	RPI_V2_GPIO_P1_18
#define GPIO_TOF_4 	RPI_V2_GPIO_P1_29
#define GPIO_TOF_5	RPI_V2_GPIO_P1_32
#define GPIO_TOF_6 	RPI_V2_GPIO_P1_31
#define GPIO_TOF_7 	RPI_V2_GPIO_P1_33
#define GPIO_TOF_8 	RPI_V2_GPIO_P1_35
#define GPIO_TOF_9 	RPI_V2_GPIO_P1_36
#define NUM_OF_TOF 8
// #define NUM_OF_TOF 9

class TOFReaderNode : public rclcpp::Node{
	public:
		TOFReaderNode();
		~TOFReaderNode();

	private:
		VL53L1X *tofSensors[NUM_OF_TOF];
		FrequencyCounter *counter;

		rclcpp::TimerBase::SharedPtr read_tof_data_timer;
		rclcpp::Publisher<minirys_interfaces::msg::TofOutput>::SharedPtr tof_data_publisher;
		minirys_interfaces::msg::TofOutput msg;
		void read_tof_data();
};