#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "FrequencyCounter/FrequencyCounter.hpp"
#include <cmath>
#include <sched.h>
#include <sys/mman.h>
#include <cstring>
#include <unistd.h>

using namespace std::chrono_literals;
#define MAX_SAFE_STACK (8*1024)

void setRTPriority() {
	// nice(-20);
	int policy = SCHED_RR;
	struct sched_param schedulerParams;
	schedulerParams.sched_priority = sched_get_priority_max(policy)-1;
	// schedulerParams.sched_priority = sched_get_priority_max(policy);
	std::cout << "[MAIN] Setting RT scheduling, priority " << schedulerParams.sched_priority << std::endl;
	if (sched_setscheduler(0, policy, &schedulerParams) == -1) {
		std::cout << "[MAIN] WARNING: Setting RT scheduling failed: " << strerror(errno) << std::endl;
		return;
	}

	if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
		std::cout << "[MAIN] WARNING: Failed to lock memory: " << strerror(errno) << std::endl;
	}
	unsigned char dummy[MAX_SAFE_STACK];
	memset(dummy, 0, MAX_SAFE_STACK);
}

int main(int argc, char * argv[]) {
	setRTPriority();
	std::cout << "Initializing ROS...\n";
	rclcpp::init(argc, argv);

	const std::string robotName = "rys";
	const std::string nodeName = "counter";
	std::chrono::microseconds loopDuration = 2000us;
	for (int i = 2; i <= argc; i+=2) {
		if (!strcmp(argv[i-1], "-f")){
			float period_s = 1/atof(argv[i]);
			unsigned long period_us = std::round(1000000*period_s);
			loopDuration = std::chrono::microseconds(period_us);
		}
	}

	auto counter = std::make_shared<FrequencyCounter>();
	// auto counter2 = std::make_shared<FrequencyCounter>();
	auto timer_callback = 
		[&counter](){
		// [&counter, &counter2](){
			counter->count();
			// counter2->count2();
		};
	auto node = rclcpp::Node::make_shared(nodeName);//, robotName, true);
	auto timer = node->create_wall_timer(loopDuration, timer_callback);

	// rclcpp::executors::MultiThreadedExecutor executor;
	rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(node);
	executor.spin();
	rclcpp::shutdown();

	return 0;
}
