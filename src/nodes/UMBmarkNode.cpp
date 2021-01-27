#include "nodes/UMBmarkNode.hpp"
// #include "minirys_drivers/srv/get_minirys_global_localization.hpp"
using namespace std::chrono_literals;

UMBmarkNode::UMBmarkNode(robot_control_data& robotControlStructure, odometry_data& odometryStructure): Node("minirys_umbmark") {	
	robotControlDataStructure = &robotControlStructure;
	odometryDataStructure = &odometryStructure;

	client = this->create_client<minirys_drivers::srv::GetMinirysGlobalLocalization>("get_minirys_global_localization");
	request = std::make_shared<minirys_drivers::srv::GetMinirysGlobalLocalization::Request>();
	request->reset = true;
	while (!client->wait_for_service(1s)) {
	    if (!rclcpp::ok()) {
			RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
			return;
	    }
	    RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
	}
	client->async_send_request(request);

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
	waitForReadings = false;

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
		if (cw_runs_completed < 5 || ccw_runs_completed < 5) {
			if (phase_of_movement % 2 == 1) { // drive to the next point
				//calculate distance from desired point
				if (cw_runs_completed < 5) position_error = sqrt(pow(cw_positions[(int)floor(phase_of_movement/2)][0] - odometryPosition[0], 2) +
									pow(cw_positions[(int)floor(phase_of_movement/2)][1] - odometryPosition[1], 2));
				else position_error = sqrt(pow(ccw_positions[(int)floor(phase_of_movement/2)][0] - odometryPosition[0], 2) +
									pow(ccw_positions[(int)floor(phase_of_movement/2)][1] - odometryPosition[1], 2));
				// RCLCPP_INFO(this->get_logger(), "Position error(Curr,Prev): %f, %f", position_error, prev_position_error);
				if (position_error > delta && position_error <= prev_position_error){ // movement not complete
					forward_speed = 10;
					prev_position_error = position_error;
				} else { // movement complete, go to next phase
					forward_speed = 0;
					prev_position_error = 999999;
					phase_of_movement++;
				}
				rotation_speed = 0;
			} else { // rotate to the next angle
				//calculate angle error from desired
				if (cw_runs_completed < 5) angle_error = abs(cw_positions[phase_of_movement/2-1][2] - odometryPosition[2]);
				else angle_error = abs(ccw_positions[phase_of_movement/2-1][2] - odometryPosition[2]);
				angle_error = (angle_error <= M_PI ? angle_error : 2*M_PI - angle_error);
				// RCLCPP_INFO(this->get_logger(), "Angle error(Curr,Prev): %f, %f", angle_error, prev_angle_error);
				if (angle_error > delta && angle_error <= prev_angle_error) { // turning not complete
					rotation_speed  = cw_runs_completed < 5 ? 5 : -5;
					prev_angle_error = angle_error;
				} else { // rotation complete, go to the next phase
					rotation_speed = 0;
					prev_angle_error = 999999;
					phase_of_movement++;
				}
				forward_speed = 0;
			}

			if (phase_of_movement == 9) { // cw_run complete, reset phase of movement
				if (cw_runs_completed < 5) {
					RCLCPP_INFO(this->get_logger(), "CW run %d completed", cw_runs_completed);
					cw_runs_completed++;
				} else {
					RCLCPP_INFO(this->get_logger(), "CW run %d completed", ccw_runs_completed);
					ccw_runs_completed++;
				}
				phase_of_movement = 1;
				compareReadings = true;
			}
		}
	}

	robotControlDataStructure->robot_control_data_access.lock();
	robotControlDataStructure->forwardSpeed = forward_speed;
	robotControlDataStructure->rotationSpeed = rotation_speed;
	robotControlDataStructure->robot_control_data_access.unlock();
	counter.count();

	if (compareReadings) {
		if (!waitForReadings) {
			auto callback = [&,this](rclcpp::Client<minirys_drivers::srv::GetMinirysGlobalLocalization>::SharedFuture inner_future)
	        { 
	            result = inner_future.get();
	            RCLCPP_INFO(this->get_logger(), "Callback executed.");
				if (result->x != 999999 && result->y != 999999 && result->theta != 999999){
					RCLCPP_INFO(this->get_logger(), "Localization:\n\tGlobal: \n%f\t%f\t%f\n\tOdometry: \n%f\t%f\t%f", result->x, result->y, result->theta, odometryPosition[0], odometryPosition[1], odometryPosition[2]);
					compareReadings = false;
					request->reset = true;
					client->async_send_request(request);

					robotControlDataStructure->robot_control_data_access.lock();
					robotControlDataStructure->setOdometryPosition = true;
					robotControlDataStructure->x = 0;
					robotControlDataStructure->y = 0;
					robotControlDataStructure->theta = 0;
					robotControlDataStructure->printRobotLocation = true;
					robotControlDataStructure->robot_control_data_access.unlock();
				}
				waitForReadings = false;
	        };
			// request = std::make_shared<minirys_drivers::srv::GetMinirysGlobalLocalization::Request>();
			request->reset = false;
			client->async_send_request(request, callback);
			waitForReadings = true;	
            RCLCPP_INFO(this->get_logger(), "Requested service.");
		}
	}
}