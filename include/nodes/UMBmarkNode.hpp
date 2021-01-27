#include "rclcpp/rclcpp.hpp"
#include "../data_structures.h"
#include "../FrequencyCounter/FrequencyCounter.hpp"
#include "minirys_drivers/srv/get_minirys_global_localization.hpp"

class UMBmarkNode : public rclcpp::Node{

	public:
		UMBmarkNode(robot_control_data&, odometry_data&);

	private:
		robot_control_data *robotControlDataStructure;
		odometry_data *odometryDataStructure;

		float period, odometryPosition[3];
		FrequencyCounter counter;
		unsigned int cw_runs_completed, ccw_runs_completed, phase_of_movement;
		int forward_speed, rotation_speed;

		const float cw_positions[4][3] = {{0.2, 0, -M_PI/2}, {0.2, -0.2, -M_PI}, {0, -0.2, M_PI/2}, {0, 0, 0}};
		const float ccw_positions[4][3] = {{0.2, 0, M_PI/2}, {0.2, 0.2, M_PI}, {0, 0.2, -M_PI/2}, {0, 0, 0}};

		float position_error, prev_position_error, angle_error, prev_angle_error;
		const float delta = 0.001;

		bool compareReadings, waitForReadings;

		rclcpp::TimerBase::SharedPtr timer;
		rclcpp::Client<minirys_drivers::srv::GetMinirysGlobalLocalization>::SharedPtr client;
		rclcpp::Client<minirys_drivers::srv::GetMinirysGlobalLocalization>::SharedRequest request;
		rclcpp::Client<minirys_drivers::srv::GetMinirysGlobalLocalization>::SharedResponse result;
		// rclcpp::Client<minirys_drivers::srv::GetMinirysGlobalLocalization>::SharedFuture future;

		void run();
};