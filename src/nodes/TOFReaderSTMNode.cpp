#include "nodes/TOFReaderSTMNode.hpp"

TOFReaderSTMNode::TOFReaderSTMNode(tof_data &structure): Node("tof_reader_stm"){
	dataStructure = &structure;

	// setting GPIO direction
	for (int i = 0; i < NUM_OF_TOF; i++) bcm2835_gpio_fsel(sensorsPins[i], BCM2835_GPIO_FSEL_OUTP);

	//disable all sensors first
	for (int i = 0; i < NUM_OF_TOF; i++) bcm2835_gpio_clr(sensorsPins[i]);	

	this->declare_parameter("sensorsUsed", rclcpp::ParameterValue(6));
	sensorsUsed = this->get_parameter("sensorsUsed").get_value<int>();

	//sensors' and sata structures
	for (int s = 0; s < sensorsUsed; ++s) {
		sensor[s] = new VL53L1_Dev_t();
		sensor[s]->I2cDevAddr = 0x29;
		data[s] = new VL53L1_RangingMeasurementData_t();
	}

	RCLCPP_INFO(this->get_logger(), "Starting TOF sensors and setting i2c addresses.");
	for (int s = 0; s < sensorsUsed; ++s)
	{
		bcm2835_gpio_set(sensorsPins[s]);
		bcm2835_delay(100);
		VL53L1_SetDeviceAddress(sensor[s], sensorsAddress[s]*2);
		sensor[s]->I2cDevAddr = sensorsAddress[s];
		bcm2835_delay(100);
	}

	RCLCPP_INFO(this->get_logger(), "Device booted.");
	for (int s = 0; s < sensorsUsed; ++s)
	{
		VL53L1_WaitDeviceBooted(sensor[s]);
	}
	bcm2835_delay(100);

	RCLCPP_INFO(this->get_logger(), "Data init.");
	for (int s = 0; s < sensorsUsed; ++s)
	{
		VL53L1_DataInit(sensor[s]);
	}
	bcm2835_delay(100);

	// static initialization
	RCLCPP_INFO(this->get_logger(), "Static init.");
	for (int s = 0; s < sensorsUsed; ++s)
	{
		VL53L1_StaticInit(sensor[s]);
	}
	bcm2835_delay(100);

	// measurement timing budget initialization
	this->declare_parameter("mtb", rclcpp::ParameterValue(20000));
	mtb = this->get_parameter("mtb").get_value<int>();
	RCLCPP_INFO(this->get_logger(), "Setting measurement timing budget.");
	for (int s = 0; s < sensorsUsed; ++s)
	{
		VL53L1_SetMeasurementTimingBudgetMicroSeconds(sensor[s], mtb);
	}
	bcm2835_delay(100);

	// inter measurement period initialization
	this->declare_parameter("imp", rclcpp::ParameterValue(24));
	imp = this->get_parameter("imp").get_value<int>();
	RCLCPP_INFO(this->get_logger(), "Setting inter measurement period.");
	for (int s = 0; s < sensorsUsed; ++s)
	{
		VL53L1_SetInterMeasurementPeriodMilliSeconds(sensor[s], imp);
	}
	bcm2835_delay(100);

	// start sensors' measurements
	RCLCPP_INFO(this->get_logger(), "Starting measurements.");
	for (int s = 0; s < sensorsUsed; ++s)
	{
		VL53L1_StartMeasurement(sensor[s]);
	}
	bcm2835_delay(100);

	this->declare_parameter("period", rclcpp::ParameterValue(100));
	read_tof_data_timer = this->create_wall_timer(
	std::chrono::milliseconds(this->get_parameter("period").get_value<int>()), std::bind(&TOFReaderSTMNode::read_tof_data, this));
}

TOFReaderSTMNode::~TOFReaderSTMNode(){
	// stop measurement process
	for (int s = 0; s < sensorsUsed; ++s)
	{
		VL53L1_StopMeasurement(sensor[s]);
		delete sensor[s];
		delete data[s];
	}
}

void TOFReaderSTMNode::read_tof_data(){
	counter.count();

	do {
		ready = 0;
		for (int s = 0; s < sensorsUsed; ++s) {
			VL53L1_GetMeasurementDataReady(sensor[s], &dataReady[s]);
			ready += dataReady[s];
	// 		printf("%d ", dataReady[s]);
		}
	// 	printf("\n");
	} while(ready != 6);

	for (int s = 0; s < sensorsUsed; ++s)
	{
		VL53L1_WaitMeasurementDataReady(sensor[s]);
		VL53L1_GetRangingMeasurementData(sensor[s], data[s]);
		VL53L1_ClearInterruptAndStartMeasurement(sensor[s]);
	}
	// RCLCPP_INFO(this->get_logger(), "%d, %d, %d, %d, %d, %d", data[0]->RangeMilliMeter, data[1]->RangeMilliMeter, data[2]->RangeMilliMeter, data[3]->RangeMilliMeter, data[4]->RangeMilliMeter, data[5]->RangeMilliMeter);
	dataStructure->tof_data_access.lock();
	for (int s = 0; s < sensorsUsed; ++s)
	{
		dataStructure->measurement[s] = (float)data[s]->RangeMilliMeter;
	}
	dataStructure->tof_data_access.unlock();
}