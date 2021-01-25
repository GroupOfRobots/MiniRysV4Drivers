#include "nodes/UMBmarkNode.hpp"
// #include "minirys_drivers/srv/get_minirys_global_localization.hpp"
using namespace std::chrono_literals;

UMBmarkNode::UMBmarkNode(robot_control_data& robotControlStructure, odometry_data& odometryStructure): Node("minirys_umbmark") {	
	robotControlDataStructure = &robotControlStructure;
	odometryDataStructure = &odometryStructure;

	client = this->create_client<minirys_drivers::srv::GetMinirysGlobalLocalization>("get_minirys_global_localization");
	auto request = std::make_shared<minirys_drivers::srv::GetMinirysGlobalLocalization::Request>();
	request->reset = true;
	auto result = client->async_send_request(request);
	if (rclcpp::spin_until_future_complete(std::shared_ptr<rclcpp::Node>(this), result) == rclcpp::executor::FutureReturnCode::SUCCESS){
		robotControlDataStructure->robot_control_data_access.lock();
		robotControlDataStructure->setOdometryPosition = true;
		robotControlDataStructure->x = result.get()->x;
		robotControlDataStructure->y = result.get()->y;
		robotControlDataStructure->theta = result.get()->theta;
		robotControlDataStructure->printRobotLocation = true;
		robotControlDataStructure->robot_control_data_access.unlock();
	}

	cw_runs_completed = 0;
	ccw_runs_completed = 0;
	phase_of_movement = 1;
	spins_to_next_phase = 200;

	this->declare_parameter("period", rclcpp::ParameterValue(10));
	period = (float)this->get_parameter("period").get_value<int>()/1000; // s
	timer = this->create_wall_timer(
	std::chrono::milliseconds(this->get_parameter("period").get_value<int>()), std::bind(&UMBmarkNode::run, this));
	RCLCPP_INFO(this->get_logger(), "UMBmark node initialized.");
}

void UMBmarkNode::run() {
	if (cw_runs_completed < 5) {
		if (phase_of_movement % 2 == 1) {
			if (spins_to_next_phase > 0) {
				// drive forward
				forward_speed = 10;
				rotation_speed = 0;
				spins_to_next_phase--;
			} else {
				//stop
				forward_speed = 0;
				rotation_speed = 0;
				phase_of_movement++;
				spins_to_next_phase = 200;
			}
		} else {
			if (spins_to_next_phase > 0) {
				// drive forward
				forward_speed = 0;
				rotation_speed = 10;
				spins_to_next_phase--;
			} else {
				//stop
				forward_speed = 0;
				rotation_speed = 0;
				phase_of_movement++;
				spins_to_next_phase = 200;
			}
		}

		if (phase_of_movement == 9){
			phase_of_movement = 1;
			cw_runs_completed++;
		}
	}

	if (phase_of_movement == 1 && spins_to_next_phase == 200) {
		auto request = std::make_shared<minirys_drivers::srv::GetMinirysGlobalLocalization::Request>();
		auto result = client->async_send_request(request);
		if (rclcpp::spin_until_future_complete(std::shared_ptr<rclcpp::Node>(this), result) == rclcpp::executor::FutureReturnCode::SUCCESS){
			RCLCPP_INFO(this->get_logger(), "%f/t%f", result.get()->x, result.get()->y);
			odometryDataStructure->odometry_data_access.lock();
			RCLCPP_INFO(this->get_logger(), "%f/t%f", odometryDataStructure->x, odometryDataStructure->y);
			odometryDataStructure->odometry_data_access.unlock();
		}

		request->reset = true;
		result = client->async_send_request(request);
		if (rclcpp::spin_until_future_complete(std::shared_ptr<rclcpp::Node>(this), result) == rclcpp::executor::FutureReturnCode::SUCCESS){
			robotControlDataStructure->robot_control_data_access.lock();
			robotControlDataStructure->setOdometryPosition = true;
			robotControlDataStructure->x = result.get()->x;
			robotControlDataStructure->y = result.get()->y;
			robotControlDataStructure->theta = result.get()->theta;
			robotControlDataStructure->printRobotLocation = true;
			robotControlDataStructure->robot_control_data_access.unlock();
		}
	}

	robotControlDataStructure->robot_control_data_access.lock();
	robotControlDataStructure->forwardSpeed = forward_speed;
	robotControlDataStructure->rotationSpeed = rotation_speed;
	robotControlDataStructure->robot_control_data_access.unlock();
	counter.count();
}