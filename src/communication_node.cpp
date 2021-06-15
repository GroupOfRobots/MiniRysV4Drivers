#include "rclcpp/rclcpp.hpp"
#include "nodes/CommunicationNode.hpp"
#include "rt.h"

int main(int argc, char const *argv[])
{
	// set real time priority for thread
	setRTPriority("Communication");

	// initiate ROS2
	rclcpp::init(argc, argv);

	// create IMU node
	auto Communication = std::make_shared<CommunicationNode>();

	// run node
	rclcpp::spin(Communication);

	// release ROS2
	rclcpp::shutdown();
	return 0;
}