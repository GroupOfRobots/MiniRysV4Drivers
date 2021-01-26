#include "nodes/UMBmarkNode.hpp"
// #include "minirys_drivers/srv/get_minirys_global_localization.hpp"
using namespace std::chrono_literals;

UMBmarkNode::UMBmarkNode(robot_control_data& robotControlStructure, odometry_data& odometryStructure): Node("minirys_umbmark") {	
	robotControlDataStructure = &robotControlStructure;
	odometryDataStructure = &odometryStructure;

	RCLCPP_INFO(this->get_logger(), "Debug0");
	client = this->create_client<minirys_drivers::srv::GetMinirysGlobalLocalization>("get_minirys_global_localization");
	auto request = std::make_shared<minirys_drivers::srv::GetMinirysGlobalLocalization::Request>();
	request->reset = true;
	auto result = client->async_send_request(request);

	robotControlDataStructure->robot_control_data_access.lock();
	robotControlDataStructure->setOdometryPosition = true;
	robotControlDataStructure->x = 0;
	robotControlDataStructure->y = 0;
	robotControlDataStructure->theta = 0;
	robotControlDataStructure->printRobotLocation = true;
	robotControlDataStructure->robot_control_data_access.unlock();

	odometryPosition[0] = 0;
	odometryPosition[1] = 0;
	odometryPosition[2] = 0;

	cw_runs_completed = 0;
	ccw_runs_completed = 0;
	phase_of_movement = 1;
	forward_speed = 0;
	rotation_speed = 0;
	position_error = 0;
	angle_error = 0;
	prev_position_error = 999999;
	prev_angle_error = 999999;

	compareReadings = false;

	RCLCPP_INFO(this->get_logger(), "Debug2");
	this->declare_parameter("period", rclcpp::ParameterValue(10));
	period = (float)this->get_parameter("period").get_value<int>()/1000; // s
	timer = this->create_wall_timer(
	std::chrono::milliseconds(this->get_parameter("period").get_value<int>()), std::bind(&UMBmarkNode::run, this));
	RCLCPP_INFO(this->get_logger(), "UMBmark node initialized.");
}

void UMBmarkNode::run() {
	odometryDataStructure->odometry_data_access.lock();
	odometryPosition[0] = odometryDataStructure->x;
	odometryPosition[1] = odometryDataStructure->y;
	odometryPosition[2] = odometryDataStructure->theta;
	odometryDataStructure->odometry_data_access.unlock();

	if (!compareReadings){
		if (cw_runs_completed < 5) {
			if (phase_of_movement % 2 == 1) { // drive to the next point
				//calculate distance from desired point
				position_error = sqrt(pow(cw_positions[(int)floor(phase_of_movement/2)][0] - odometryPosition[0], 2) +
									pow(cw_positions[(int)floor(phase_of_movement/2)][1] - odometryPosition[1], 2));
				RCLCPP_INFO(this->get_logger(), "Position error: %f, %f", position_error, prev_position_error);
				if (position_error > delta && position_error <= prev_position_error){ // movement not complete
					forward_speed = 10;
					prev_position_error = position_error;
				} else { // movement complete, go to next phase
					forward_speed = 0;
					prev_position_error = 999999;
					phase_of_movement++;
				}
				rotation_speed = 0;
			} else { // rotatr to the next angle
				//calculate angle error from desired
				angle_error = abs(cw_positions[phase_of_movement/2-1][2] - odometryPosition[2]);
				angle_error = (angle_error <= M_PI ? angle_error : 2*M_PI - angle_error);
				RCLCPP_INFO(this->get_logger(), "Angle error: %f, %f", angle_error, prev_angle_error);
				if (angle_error > delta && angle_error <= prev_angle_error) { // turning not complete
					rotation_speed = 5;
					prev_angle_error = angle_error;
				} else { // rotation complete, go to the next phase
					rotation_speed = 0;
					prev_angle_error = 999999;
					phase_of_movement++;
				}
				forward_speed = 0;
			}

			if (phase_of_movement == 9) { // cw_run complete, reset phase of movement
				cw_runs_completed++;
				phase_of_movement = 1;
				compareReadings = true;
			}
		} else if (ccw_runs_completed < 5) {
			if (phase_of_movement % 2 == 1) { // drive to the next point
				//calculate distance from desired point
				position_error = sqrt(pow(cw_positions[(int)floor(phase_of_movement/2)][0] - odometryPosition[0], 2) +
									pow(cw_positions[(int)floor(phase_of_movement/2)][1] - odometryPosition[1], 2));
				RCLCPP_INFO(this->get_logger(), "Position error: %f, %f", position_error, prev_position_error);
				if (position_error > delta && position_error <= prev_position_error){ // movement not complete
					forward_speed = 10;
					prev_position_error = position_error;
				} else { // movement complete, go to next phase
					forward_speed = 0;
					prev_position_error = 999999;
					phase_of_movement++;
				}
				rotation_speed = 0;
			} else { // rotatr to the next angle
				//calculate angle error from desired
				angle_error = abs(cw_positions[phase_of_movement/2-1][2] - odometryPosition[2]);
				angle_error = (angle_error <= M_PI ? angle_error : 2*M_PI - angle_error);
				RCLCPP_INFO(this->get_logger(), "Angle error: %f, %f", angle_error, prev_angle_error);
				if (angle_error > delta && angle_error <= prev_angle_error) { // turning not complete
					rotation_speed = -5;
					prev_angle_error = angle_error;
				} else { // rotation complete, go to the next phase
					rotation_speed = 0;
					prev_angle_error = 999999;
					phase_of_movement++;
				}
				forward_speed = 0;
			}

			if (phase_of_movement == 9) { // cw_run complete, reset phase of movement
				cw_runs_completed++;
				phase_of_movement = 1;
				compareReadings = true;
			}
		}
	}

	if (compareReadings) {
		auto request = std::make_shared<minirys_drivers::srv::GetMinirysGlobalLocalization::Request>();
		auto result = client->async_send_request(request);
		// if (rclcpp::spin_until_future_complete(std::shared_ptr<rclcpp::Node>(this), result) == rclcpp::executor::FutureReturnCode::SUCCESS){
			if (result.get()->x != 999999 && result.get()->y != 999999 && result.get()->theta != 999999){
				RCLCPP_INFO(this->get_logger(), "Global: %f/t%f\nOdometry: %f/t%f", result.get()->x, result.get()->y, odometryPosition[0], odometryPosition[1]);
				compareReadings = false;
				request->reset = true;
				result = client->async_send_request(request);
				// if (rclcpp::spin_until_future_complete(std::shared_ptr<rclcpp::Node>(this), result) == rclcpp::executor::FutureReturnCode::SUCCESS){
					robotControlDataStructure->robot_control_data_access.lock();
					robotControlDataStructure->setOdometryPosition = true;
					robotControlDataStructure->x = 0;
					robotControlDataStructure->y = 0;
					robotControlDataStructure->theta = 0;
					robotControlDataStructure->printRobotLocation = true;
					robotControlDataStructure->robot_control_data_access.unlock();
				// }
			}
		// }
	}

	robotControlDataStructure->robot_control_data_access.lock();
	robotControlDataStructure->forwardSpeed = forward_speed;
	robotControlDataStructure->rotationSpeed = rotation_speed;
	robotControlDataStructure->robot_control_data_access.unlock();
	counter.count();
}