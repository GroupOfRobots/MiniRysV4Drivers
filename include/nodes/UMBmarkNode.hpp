#include "rclcpp/rclcpp.hpp"
#include "../data_structures.h"
#include "../FrequencyCounter/FrequencyCounter.hpp"
#include "minirys_interfaces/srv/get_minirys_global_localization.hpp"
#include "minirys_interfaces/msg/minirys_output.hpp"
#include "minirys_interfaces/msg/minirys_input.hpp"
#include <iostream>
#include <fstream>

class UMBmarkNode : public rclcpp::Node{

	public:
		UMBmarkNode();
		~UMBmarkNode();

	private:

		float period, odometryPosition[3];
		FrequencyCounter counter;
		unsigned int cw_runs_completed, ccw_runs_completed, periods_completed, phase_of_movement;
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
		rclcpp::Subscription<minirys_interfaces::msg::MinirysOutput>::SharedPtr minirys_subscriber;
		rclcpp::Publisher<minirys_interfaces::msg::MinirysInput>::SharedPtr minirys_publisher;
		minirys_interfaces::msg::MinirysInput minirys_message;

		std::ofstream outputFile;

		void run();
		void minirysCallback(const minirys_interfaces::msg::MinirysOutput::SharedPtr msg);
};