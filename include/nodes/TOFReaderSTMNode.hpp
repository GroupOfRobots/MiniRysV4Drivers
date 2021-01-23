#include "rclcpp/rclcpp.hpp"
#include "../data_structures.h"
#include "../FrequencyCounter/FrequencyCounter.hpp"
#include "../vl53l1x_stm/vl53l1_api.h"
#include "../vl53l1x_stm/vl53l1_platform.h"
#include "../bcm/bcm2835.h"

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

class TOFReaderSTMNode : public rclcpp::Node{

	public:
		TOFReaderSTMNode(tof_data &structure);
		~TOFReaderSTMNode();

	private:
		VL53L1_Dev_t *sensor[NUM_OF_TOF];
		VL53L1_RangingMeasurementData_t *data[NUM_OF_TOF];
		uint8_t dataReady[NUM_OF_TOF];
		uint8_t ready;
		tof_data *dataStructure = NULL;
		FrequencyCounter counter;
		int sensorsUsed, mtb, imp;
		const uint8_t sensorsPins[NUM_OF_TOF] = {GPIO_TOF_1, GPIO_TOF_3, GPIO_TOF_4, GPIO_TOF_5, GPIO_TOF_6, GPIO_TOF_7, GPIO_TOF_8, GPIO_TOF_9};
		const uint8_t sensorsAddress[NUM_OF_TOF] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37};
		rclcpp::TimerBase::SharedPtr read_tof_data_timer;

		void read_tof_data();
	
};