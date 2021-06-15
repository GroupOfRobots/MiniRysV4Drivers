#include "rclcpp/rclcpp.hpp"
#include "../data_structures.h"
#include "../FrequencyCounter/FrequencyCounter.hpp"
#include "minirys_interfaces/msg/motors_controller_output.hpp"
#include "minirys_interfaces/srv/set_odometry_position.hpp"
#include "nav_msgs/msg/odometry.hpp"

#define STEPS_TO_RAD 2*M_PI/200 // muliply by wheel radius to receive distance

class OdometryCalculatorNode : public rclcpp::Node{
	public:
		OdometryCalculatorNode();
		~OdometryCalculatorNode();
	private:
		double period;
		double x, y, angle;
		double wheelDistance, wheelRadius_l, wheelRadius_r;
		double speed_l, setSpeed_l, previousSetSpeed_l, speed_r, setSpeed_r, previousSetSpeed_r;
		double acceleration;
		double currentTime, lastMessageTime;
		double phaseDuration;
		double accelerationTime_l, accelerationTime_r, distance_l, distance_r;

		FrequencyCounter *counter;
		rclcpp::TimerBase::SharedPtr odometry_calculation_timer;
		rclcpp::Subscription<minirys_interfaces::msg::MotorsControllerOutput>::SharedPtr motors_controller_subscriber;
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher;
		rclcpp::Service<minirys_interfaces::srv::SetOdometryPosition>::SharedPtr set_position_service;
		nav_msgs::msg::Odometry msg;

		void receiveCurrentSetSpeeds(const minirys_interfaces::msg::MotorsControllerOutput::SharedPtr msg);
		void updatePosition();
		void printLocation();
		void cropAngle();
		void setOdometryPosition(const std::shared_ptr<minirys_interfaces::srv::SetOdometryPosition::Request> request,
			std::shared_ptr<minirys_interfaces::srv::SetOdometryPosition::Response> response);
};