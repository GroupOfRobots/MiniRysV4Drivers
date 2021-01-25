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

		float period;
		FrequencyCounter counter;
		unsigned int cw_runs_completed, ccw_runs_completed, phase_of_movement, spins_to_next_phase, forward_speed, rotation_speed;

		rclcpp::TimerBase::SharedPtr timer;
		rclcpp::Client<minirys_drivers::srv::GetMinirysGlobalLocalization>::SharedPtr client;

		void run();
};