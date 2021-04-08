#include "rclcpp/rclcpp.hpp"
#include "../data_structures.h"
#include "../FrequencyCounter/FrequencyCounter.hpp"
#include "minirys_interfaces/srv/get_minirys_global_localization.hpp"
#include <iostream>
#include <fstream>

class UMBmarkNode : public rclcpp::Node{

	public:
		UMBmarkNode(robot_control_data&, odometry_data&);
		~UMBmarkNode();

	private:
		robot_control_data *robotControlDataStructure;
		odometry_data *odometryDataStructure;

		float period, odometryPosition[3];
		FrequencyCounter counter;
		unsigned int cw_runs_completed, ccw_runs_completed, periods, phase_of_movement;
		float forward_speed, rotation_speed;

		const unsigned int forward_periods = 1447;
		const unsigned int turn_periods = 1443;

		// const float cw_positions[4][3] = {{0.2, 0, -M_PI/2}, {0.2, -0.2, -M_PI}, {0, -0.2, M_PI/2}, {0, 0, 0}};
		// const float ccw_positions[4][3] = {{0.2, 0, M_PI/2}, {0.2, 0.2, M_PI}, {0, 0.2, -M_PI/2}, {0, 0, 0}};

		// float position_error, prev_position_error, angle_error, prev_angle_error;
		// const float delta = 0.0001;

		bool compareReadings, waitForReadings;

		rclcpp::TimerBase::SharedPtr timer;
		rclcpp::Client<minirys_interfaces::srv::GetMinirysGlobalLocalization>::SharedPtr client;
		rclcpp::Client<minirys_interfaces::srv::GetMinirysGlobalLocalization>::SharedRequest request;
		rclcpp::Client<minirys_interfaces::srv::GetMinirysGlobalLocalization>::SharedResponse result;

		std::ofstream outputFile;

		void run();
};