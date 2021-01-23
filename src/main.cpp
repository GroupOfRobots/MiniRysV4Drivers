#include <memory>
#include <sched.h>
#include <sys/mman.h>
#include "rclcpp/rclcpp.hpp"
#include "nodes/ImuReaderNode.hpp"
#include "nodes/OdometryCalculatorNode.hpp"
#include "nodes/JoyconReceiverNode.hpp"
#include "nodes/MotorsControllerNode.hpp"
// #include "nodes/TOFReaderNode.hpp"
// #include "nodes/TOFReaderSTMNode.hpp"

// bool endProcess = false;

// void sigintHandler(int signum) {
// 	if (signum == SIGINT) {
// 		endProcess = true;
// 	}
// }

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
}

int main(int argc, char * argv[]) {
	setbuf(stdout, nullptr);
	setRTPriority();
	if (bcm2835_init() == 0) {
		fprintf(stderr, "Not able to init the bmc2835 library\n");
		return -1;
	}
	rclcpp::init(argc, argv);
	// signal(SIGINT, sigintHandler);
	rclcpp::executors::SingleThreadedExecutor executor;
	// rclcpp::executors::MultiThreadedExecutor executor;

	robot_control_data robot_control_data_structure;
	imu_data imu_data_structure;
	motor_data motor_data_structure;
	// tof_data tof_data_structure;

	auto JoyconReceiver = std::make_shared<JoyconReceiverNode>(std::ref(robot_control_data_structure));
	auto ImuReader = std::make_shared<ImuReaderNode>(std::ref(imu_data_structure));
	auto MotorsController = std::make_shared<MotorsControllerNode>(std::ref(imu_data_structure), std::ref(robot_control_data_structure), std::ref(motor_data_structure));
	auto OdometryCalculator = std::make_shared<OdometryCalculatorNode>(std::ref(motor_data_structure), std::ref(robot_control_data_structure));
	// auto TOFReader = std::make_shared<TOFReaderNode>(std::ref(tof_data_structure));
	// auto TOFReaderSTM = std::make_shared<TOFReaderSTMNode>(std::ref(tof_data_structure));

	executor.add_node(JoyconReceiver);
	executor.add_node(ImuReader);
	executor.add_node(MotorsController);
	executor.add_node(OdometryCalculator);
	// executor.add_node(TOFReader);
	// executor.add_node(TOFReaderSTM);

	executor.spin();
	rclcpp::shutdown();
	return 0;
}