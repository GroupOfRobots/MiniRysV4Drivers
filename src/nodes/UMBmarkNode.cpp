#include "nodes/UMBmarkNode.hpp"
#include <cmath>
#include <algorithm>
using namespace std::chrono_literals;

UMBmarkNode::UMBmarkNode(): Node("minirys_umbmark") {

	client = this->create_client<minirys_interfaces::srv::GetMinirysGlobalLocalization>("get_minirys_global_localization");
	request = std::make_shared<minirys_interfaces::srv::GetMinirysGlobalLocalization::Request>();
	request->reset = true;
	while (!client->wait_for_service(1s)) {
	    if (!rclcpp::ok()) {
			RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
			return;
	    }
	    RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
	}
	client->async_send_request(request);

	odometryPosition[0] = 0;
	odometryPosition[1] = 0;
	odometryPosition[2] = 0;

	cw_runs_completed = 0;
	ccw_runs_completed = 0;
	phase_of_movement = 1;
	periods_completed = 0;
	forward_speed = 0;
	rotation_speed = 0;
	// position_error = 0;
	// angle_error = 0;
	// prev_position_error = 999999;
	// prev_angle_error = 999999;

	compareReadings = false;
	waitForReadings = false;

	minirys_subscriber = this->create_subscription<minirys_interfaces::msg::MinirysOutput>("minirys_output", 10, std::bind(&UMBmarkNode::minirysCallback, this, std::placeholders::_1));
	minirys_publisher = this->create_publisher<minirys_interfaces::msg::MinirysInput>("minirys_input", 10);
	minirys_message = minirys_interfaces::msg::MinirysInput();
	minirys_message.new_odometry_position.position.x = 0;
	minirys_message.new_odometry_position.position.y = 0;
	minirys_message.new_odometry_position.orientation.w = 1;
	minirys_message.new_odometry_position.orientation.x = 0;
	minirys_message.new_odometry_position.orientation.y = 0;
	minirys_message.new_odometry_position.orientation.z = 0;


	this->declare_parameter("filename", rclcpp::ParameterValue("UMBmark_output"));
	outputFile.open(this->get_parameter("filename").get_value<std::string>());

	this->declare_parameter("period", rclcpp::ParameterValue(10));
	period = (float)this->get_parameter("period").get_value<int>()/1000; // s
	timer = this->create_wall_timer(
	std::chrono::milliseconds(this->get_parameter("period").get_value<int>()), std::bind(&UMBmarkNode::run, this));
	RCLCPP_INFO(this->get_logger(), "UMBmark node initialized.");
}

UMBmarkNode::~UMBmarkNode() {
	outputFile.close();
}

void UMBmarkNode::minirysCallback(const minirys_interfaces::msg::MinirysOutput::SharedPtr msg) {
	odometryPosition[0] = msg->odometry.pose.pose.position.x;
	odometryPosition[1] = msg->odometry.pose.pose.position.y;
	odometryPosition[2] = atan2(2*(msg->odometry.pose.pose.orientation.w*msg->odometry.pose.pose.orientation.z+msg->odometry.pose.pose.orientation.x*msg->odometry.pose.pose.orientation.y),
		1-2*(pow(msg->odometry.pose.pose.orientation.y, 2)+pow(msg->odometry.pose.pose.orientation.z, 2)));
}

void UMBmarkNode::run() {
	counter.count();
	// shutdown program after completing 5 runs each way
	if (cw_runs_completed == 5 && ccw_runs_completed == 5 && !compareReadings) rclcpp::shutdown();

	if (!compareReadings){
		if (cw_runs_completed < 5 || ccw_runs_completed < 5) {
			if (phase_of_movement % 2 == 1) { // drive to the next point
				if (periods_completed < forward_periods){// && position_error <= prev_position_error){ // movement not complete
					forward_speed = 8;
					periods_completed++;
				} else { // movement complete, go to next phase
					forward_speed = 0;
					periods_completed = 0;
					phase_of_movement++;
				}
				rotation_speed = 0;
			} else { // rotate to the next angle
				if (periods_completed < turn_periods) { // turning not complete
					rotation_speed  = cw_runs_completed < 5 ? 4 : -4;
					periods_completed++;
				} else { // rotation complete, go to the next phase
					rotation_speed = 0;
					periods_completed = 0;
					phase_of_movement++;
				}
				forward_speed = 0;
			}

			if (phase_of_movement == 9) { // cw_run complete, reset phase of movement
				if (cw_runs_completed < 5) {
					RCLCPP_INFO(this->get_logger(), "CW run %d completed", cw_runs_completed);
					cw_runs_completed++;
				} else {
					RCLCPP_INFO(this->get_logger(), "CCW run %d completed", ccw_runs_completed);
					ccw_runs_completed++;
				}
				phase_of_movement = 1;
				compareReadings = true;
			}
		}
	}

	// send steering information
	minirys_message.motor_control.forward_speed = forward_speed;
	minirys_message.motor_control.rotation_speed = rotation_speed;
	minirys_publisher->publish(minirys_message);

	if (compareReadings) {
		if (!waitForReadings) {
			auto callback = [&,this](rclcpp::Client<minirys_interfaces::srv::GetMinirysGlobalLocalization>::SharedFuture inner_future)
	        { 
	            result = inner_future.get();
	            RCLCPP_INFO(this->get_logger(), "Callback executed.");
	            // check if absolute position was correctly calculated
				if (result->x != 999999 && result->y != 999999 && result->theta != 999999){
					RCLCPP_INFO(this->get_logger(), "Localization:\n\tGlobal: \n%f\t%f\t%f\n\tOdometry: \n%f\t%f\t%f", result->x, result->y, result->theta, odometryPosition[0], odometryPosition[1], odometryPosition[2]);
					// save to file
					outputFile << result->x << " " << result->y << " " << result->theta << " " << odometryPosition[0] << " " << odometryPosition[1] << " " << odometryPosition[2] << "\n";
					compareReadings = false;
					// reset absolute position
					request->reset = true;
					client->async_send_request(request);
					// reset odometry position
					minirys_message.set_new_odometry_position = true;
					minirys_publisher->publish(minirys_message);
					minirys_message.set_new_odometry_position = false;
				}
				waitForReadings = false;
	        };
	        // send request
			request->reset = false;
			client->async_send_request(request, callback);
			waitForReadings = true;	
            RCLCPP_INFO(this->get_logger(), "Requested service.");
		}
	}
}