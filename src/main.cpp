#include "rclcpp/rclcpp.hpp"
#include "nodes/ImuReaderNode.hpp"
#include "nodes/OdometryCalculatorNode.hpp"
#include "nodes/JoyconReceiverNode.hpp"
#include "nodes/MotorsControllerNode.hpp"
#include "nodes/UMBmarkNode.hpp"
#include "nodes/CommunicationNode.hpp"
#include "nodes/TOFReaderNode.hpp"
#include "nodes/TOFReaderSTMNode.hpp"
#include "rt.h"

int main(int argc, char * argv[]) {
	setRTPriority();
	if (bcm2835_init() == 0) {
		fprintf(stderr, "Not able to init the bmc2835 library\n");
		return -1;
	}
	rclcpp::init(argc, argv);
	rclcpp::executors::MultiThreadedExecutor executor;

	auto ImuReader = std::make_shared<ImuReaderNode>();
	auto MotorsController = std::make_shared<MotorsControllerNode>();
	auto OdometryCalculator = std::make_shared<OdometryCalculatorNode>();
	// auto TOFReader = std::make_shared<TOFReaderNode>();
	// auto TOFReaderSTM = std::make_shared<TOFReaderSTMNode>();
	auto Communication = std::make_shared<CommunicationNode>();

	executor.add_node(ImuReader);
	executor.add_node(MotorsController);
	executor.add_node(OdometryCalculator);
	// executor.add_node(TOFReader);
	// executor.add_node(TOFReaderSTM);
	executor.add_node(Communication);

	executor.spin();
	rclcpp::shutdown();
	return 0;
}