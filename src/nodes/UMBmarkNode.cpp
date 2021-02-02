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
	// robotControlDataStructure->printRobotLocation = true;
	robotControlDataStructure->robot_control_data_access.unlock();

	odometryPosition[0] = 0;
	odometryPosition[1] = 0;
	odometryPosition[2] = 0;

	cw_runs_completed = 0;
	ccw_runs_completed = 0;
	phase_of_movement = 1;
	periods = 0;
	forward_speed = 0;
	rotation_speed = 0;
	// position_error = 0;
	// angle_error = 0;
	// prev_position_error = 999999;
	// prev_angle_error = 999999;

	compareReadings = false;
	waitForReadings = false;

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

void UMBmarkNode::run() {
	counter.count();
	// shutdown program after completing 5 runs each way
	if (cw_runs_completed == 5 && ccw_runs_completed == 5 && !compareReadings) rclcpp::shutdown();

	if (!compareReadings){
		if (cw_runs_completed < 5 || ccw_runs_completed < 5) {
			if (phase_of_movement % 2 == 1) { // drive to the next point
				if (periods < forward_periods){// && position_error <= prev_position_error){ // movement not complete
					forward_speed = 8;
					periods++;
				} else { // movement complete, go to next phase
					forward_speed = 0;
					periods = 0;
					phase_of_movement++;
				}
				rotation_speed = 0;
			} else { // rotate to the next angle
				if (periods < turn_periods) { // turning not complete
					rotation_speed  = cw_runs_completed < 5 ? 4 : -4;
					periods++;
				} else { // rotation complete, go to the next phase
					rotation_speed = 0;
					periods = 0;
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
	robotControlDataStructure->robot_control_data_access.lock();
	robotControlDataStructure->forwardSpeed = forward_speed;
	robotControlDataStructure->rotationSpeed = rotation_speed;
	robotControlDataStructure->robot_control_data_access.unlock();

	if (compareReadings) {
		if (!waitForReadings) {
			auto callback = [&,this](rclcpp::Client<minirys_drivers::srv::GetMinirysGlobalLocalization>::SharedFuture inner_future)
	        { 
	            result = inner_future.get();
	            RCLCPP_INFO(this->get_logger(), "Callback executed.");
	            // check if absolute position was correctly calculated
				if (result->x != 999999 && result->y != 999999 && result->theta != 999999){
					// get odometry position
					odometryDataStructure->odometry_data_access.lock();
					odometryPosition[0] = odometryDataStructure->x;
					odometryPosition[1] = odometryDataStructure->y;
					odometryPosition[2] = odometryDataStructure->theta;
					odometryDataStructure->odometry_data_access.unlock();
					RCLCPP_INFO(this->get_logger(), "Localization:\n\tGlobal: \n%f\t%f\t%f\n\tOdometry: \n%f\t%f\t%f", result->x, result->y, result->theta, odometryPosition[0], odometryPosition[1], odometryPosition[2]);
					// save to file
					outputFile << result->x << " " << result->y << " " << result->theta << " " << odometryPosition[0] << " " << odometryPosition[1] << " " << odometryPosition[2] << "\n";
					compareReadings = false;
					// reset absolute position
					request->reset = true;
					client->async_send_request(request);
					// reset odometry position
					robotControlDataStructure->robot_control_data_access.lock();
					robotControlDataStructure->setOdometryPosition = true;
					robotControlDataStructure->x = 0;
					robotControlDataStructure->y = 0;
					robotControlDataStructure->theta = 0;
					robotControlDataStructure->robot_control_data_access.unlock();
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