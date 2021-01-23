#include "nodes/TOFReaderNode.hpp"

TOFReaderNode::TOFReaderNode(tof_data& structure): Node("tof_reader"){

	dataStructure = &structure;

	bcm2835_gpio_fsel(GPIO_TOF_1, BCM2835_GPIO_FSEL_OUTP);
	// bcm2835_gpio_fsel(GPIO_TOF_2, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_3, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_4, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_5, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_6, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_7, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_8, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_TOF_9, BCM2835_GPIO_FSEL_OUTP);

	//disable all sensors first
	bcm2835_gpio_clr(GPIO_TOF_1); // górny
	// bcm2835_gpio_clr(GPIO_TOF_2); // górny
	bcm2835_gpio_clr(GPIO_TOF_3);
	bcm2835_gpio_clr(GPIO_TOF_4);
	bcm2835_gpio_clr(GPIO_TOF_5);
	bcm2835_gpio_clr(GPIO_TOF_6);
	bcm2835_gpio_clr(GPIO_TOF_7);
	bcm2835_gpio_clr(GPIO_TOF_8);
	bcm2835_gpio_clr(GPIO_TOF_9);

	bcm2835_i2c_begin(); //begin I2C
	// bcm2835_i2c_set_baudrate(40000);

	//enable sensor one and change address
	bcm2835_gpio_set(GPIO_TOF_1);
	tofSensors[0] = new VL53L1X(VL53L1X::Medium,0x29);
	tofSensors[0]->setAddress(0x30);
	RCLCPP_INFO(this->get_logger(), "TOF sensor one started at: 0x30");

	// bcm2835_gpio_set(GPIO_TOF_2);
	bcm2835_gpio_set(GPIO_TOF_9);
	tofSensors[1] = new VL53L1X(VL53L1X::Medium,0x29);
	tofSensors[1]->setAddress(0x31);
	RCLCPP_INFO(this->get_logger(), "TOF sensor two started at: 0x31");

	bcm2835_gpio_set(GPIO_TOF_3);
	tofSensors[2] = new VL53L1X(VL53L1X::Medium,0x29);
	tofSensors[2]->setAddress(0x32);
	RCLCPP_INFO(this->get_logger(), "TOF sensor three started at: 0x32");

	bcm2835_gpio_set(GPIO_TOF_4);
	tofSensors[3] = new VL53L1X(VL53L1X::Medium,0x29);
	tofSensors[3]->setAddress(0x33);
	RCLCPP_INFO(this->get_logger(), "TOF sensor four started at: 0x33");

	bcm2835_gpio_set(GPIO_TOF_5);
	tofSensors[4] = new VL53L1X(VL53L1X::Medium,0x29);
	tofSensors[4]->setAddress(0x34);
	RCLCPP_INFO(this->get_logger(), "TOF sensor five started at: 0x34");

	bcm2835_gpio_set(GPIO_TOF_6);
	tofSensors[5] = new VL53L1X(VL53L1X::Medium,0x29);
	tofSensors[5]->setAddress(0x35);
	RCLCPP_INFO(this->get_logger(), "TOF sensor six started at: 0x35");

	bcm2835_gpio_set(GPIO_TOF_7);
	tofSensors[6] = new VL53L1X(VL53L1X::Medium,0x29);
	tofSensors[6]->setAddress(0x36);
	RCLCPP_INFO(this->get_logger(), "TOF sensor seven started at: 0x36");

	bcm2835_gpio_set(GPIO_TOF_8);
	tofSensors[7] = new VL53L1X(VL53L1X::Medium,0x29);
	tofSensors[7]->setAddress(0x37);
	RCLCPP_INFO(this->get_logger(), "TOF sensor eight started at: 0x37");

	// bcm2835_gpio_set(GPIO_TOF_9);
	// tofSensors[8] = new VL53L1X(VL53L1X::Medium,0x29);
	// tofSensors[8]->setAddress(0x38);
	// RCLCPP_INFO(this->get_logger(), "TOF sensor nine started at: 0x38");

	this->declare_parameter("period", rclcpp::ParameterValue(100));
	for(int i = 0; i < NUM_OF_TOF; i++){
		tofSensors[i]->startContinuous(this->get_parameter("period").get_value<int>());
	}
	read_tof_data_timer = this->create_wall_timer(
	std::chrono::milliseconds(this->get_parameter("period").get_value<int>()), std::bind(&TOFReaderNode::read_tof_data, this));
}

TOFReaderNode::~TOFReaderNode() {
	for(int i = 0; i < NUM_OF_TOF; i++){
		tofSensors[i]->disable();
		delete tofSensors[i];
	}
}

void TOFReaderNode::read_tof_data() {
	counter.count();
	dataStructure->tof_data_access.lock();
	for(int i = 0; i < NUM_OF_TOF; i++){
		dataStructure->measurement[i] = tofSensors[i]->readData(1);
	}
	dataStructure->tof_data_access.unlock();
}