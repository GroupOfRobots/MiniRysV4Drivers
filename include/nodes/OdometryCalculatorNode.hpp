#include "rclcpp/rclcpp.hpp"
#include "../data_structures.h"
#include "../FrequencyCounter/FrequencyCounter.hpp"
#include "minirys_interfaces/msg/motors_controller_output.hpp"
#include "nav_msgs/msg/odometry.hpp"

#define STEPS_TO_REVS 2*M_PI/200 // muliply by wheel radius to receive distance

class OdometryCalculatorNode : public rclcpp::Node{
	public:
		OdometryCalculatorNode();
		~OdometryCalculatorNode();
	private:
		float period;
		float x, y, angle;
		float wheelDistance, wheelRadius_l, wheelRadius_r;
		float speed_l, setSpeed_l, previousSetSpeed_l, speed_r, setSpeed_r, previousSetSpeed_r;
		float acceleration;
		int64_t currentTime, lastMessageTime;
		float phaseDuration;
		float accelerationTime_l, accelerationTime_r, distance_l, distance_r;

		FrequencyCounter *counter;
		rclcpp::TimerBase::SharedPtr odometry_calculation_timer;
		rclcpp::Subscription<minirys_interfaces::msg::MotorsControllerOutput>::SharedPtr motors_controller_subscriber;
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher;
		nav_msgs::msg::Odometry msg;

		void receiveCurrentSetSpeeds(const minirys_interfaces::msg::MotorsControllerOutput::SharedPtr msg);
		void updatePosition();
		void printLocation();
		void cropAngle();
		void setPosition(float, float, float);
};