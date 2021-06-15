#include "rclcpp/rclcpp.hpp"
#include "nodes/TOFReaderNode.hpp"
#include "rt.h"

int main(int argc, char const *argv[])
{
	// set real time priority for thread
	setRTPriority("TOF");

	// initiate communication interfaces
	if (bcm2835_init() == 0) {
		fprintf(stderr, "Not able to init the bmc2835 library\n");
		return -1;
	}

	// initiate ROS2
	rclcpp::init(argc, argv);

	// create IMU node
	auto TOFReader = std::make_shared<TOFReaderNode>();

	// run node
	rclcpp::spin(TOFReader);

	// release ROS2
	rclcpp::shutdown();
	return 0;
}