#include "rclcpp/rclcpp.hpp"
#include "nodes/OdometryCalculatorNode.hpp"
#include "rt.h"

int main(int argc, char const *argv[])
{
	// set real time priority for thread
	setRTPriority("Odometry");

	// initiate ROS2
	rclcpp::init(argc, argv);

	// create IMU node
	auto odometry = std::make_shared<OdometryCalculatorNode>();

	// run node
	rclcpp::spin(odometry);

	// release ROS2
	rclcpp::shutdown();
	return 0;
}