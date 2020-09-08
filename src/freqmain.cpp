#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "FrequencyCounter/FrequencyCounter.hpp"
#include <cmath>
#include <sched.h>
#include <sys/mman.h>
#include <cstring>

using namespace std::chrono_literals;

void setRTPriority() {
	struct sched_param schedulerParams;
	schedulerParams.sched_priority = sched_get_priority_max(SCHED_RR)-1;
	std::cout << "[MAIN] Setting RT scheduling, priority " << schedulerParams.sched_priority << std::endl;
	if (sched_setscheduler(0, SCHED_RR, &schedulerParams) == -1) {
		std::cout << "[MAIN] WARNING: Setting RT scheduling failed: " << strerror(errno) << std::endl;
		return;
	}

	if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
		std::cout << "[MAIN] WARNING: Failed to lock memory: " << strerror(errno) << std::endl;
	}
}

int main(int argc, char * argv[]) {
	std::cout << "Initializing ROS...\n";
	rclcpp::init(argc, argv);

	const std::string robotName = "rys";
	const std::string nodeName = "counter";
	std::chrono::microseconds loopDuration = 2000us;
	for (int i = 2; i <= argc; i+=2) {
		if (!strcmp(argv[i-1], "-f")){
			float period_s = 1/atof(argv[i]);
			unsigned long period_us = std::round(1000000*period_s);
			std::cout << period_s << " " << period_us << std::endl;
			loopDuration = std::chrono::microseconds(period_us);
		}
	}

	auto counter = std::make_shared<FrequencyCounter>();
	auto timer_callback = 
		[&counter](){
			counter->count();
		};
	auto node = rclcpp::Node::make_shared(nodeName);//, robotName, true);
	auto timer = node->create_wall_timer(loopDuration, timer_callback);

	setRTPriority();
	// rclcpp::spin(node);
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(node);
	executor.spin();
	rclcpp::shutdown();

	return 0;
}
