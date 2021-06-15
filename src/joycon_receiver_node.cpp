#include "rclcpp/rclcpp.hpp"
#include "nodes/JoyconReceiverNode.hpp"
#include "rt.h"

int main(int argc, char const *argv[])
{
	// set real time priority for thread
	setRTPriority("Odometry");

	// initiate ROS2
	rclcpp::init(argc, argv);

	// create IMU node
	auto joycon = std::make_shared<JoyconReceiverNode>();

	// run node
	rclcpp::spin(joycon);

	// release ROS2
	rclcpp::shutdown();
	return 0;
}