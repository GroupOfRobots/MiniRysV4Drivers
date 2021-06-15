#include "rclcpp/rclcpp.hpp"
#include "nodes/UMBmarkNode.hpp"
#include "rt.h"

int main(int argc, char const *argv[])
{
	// set real time priority for thread
	setRTPriority("UMBmark");

	// initiate ROS2
	rclcpp::init(argc, argv);

	// create IMU node
	auto UMBmark = std::make_shared<UMBmarkNode>();

	// run node
	rclcpp::spin(UMBmark);

	// release ROS2
	rclcpp::shutdown();
	return 0;
}