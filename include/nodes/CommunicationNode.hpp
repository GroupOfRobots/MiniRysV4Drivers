#include "rclcpp/rclcpp.hpp"
#include "../FrequencyCounter/FrequencyCounter.hpp"
#include "minirys_interfaces/msg/imu_output.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "minirys_interfaces/msg/tof_output.hpp"
#include "minirys_interfaces/msg/motors_controller_output.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "minirys_interfaces/msg/minirys_output.hpp"
#include "minirys_interfaces/msg/minirys_input.hpp"
#include "minirys_interfaces/srv/set_odometry_position.hpp"
#include "minirys_interfaces/msg/motors_control.hpp"

class CommunicationNode : public rclcpp::Node{
	public:
		CommunicationNode();
		~CommunicationNode();

	private:
		FrequencyCounter *counter;

		rclcpp::TimerBase::SharedPtr communication_timer;
		void sendData();

		rclcpp::Publisher<minirys_interfaces::msg::MinirysOutput>::SharedPtr minirys_data_publisher;
		minirys_interfaces::msg::MinirysOutput outputMessage;
		rclcpp::Publisher<minirys_interfaces::msg::MotorsControl>::SharedPtr motors_control_publisher;
		minirys_interfaces::msg::MotorsControl motorsMessage;

		rclcpp::Subscription<minirys_interfaces::msg::ImuOutput>::SharedPtr imu_data_subscriber;
		rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr temperature_subscriber;
		rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_subscriber;
		rclcpp::Subscription<minirys_interfaces::msg::TofOutput>::SharedPtr tof_data_subscriber;
		rclcpp::Subscription<minirys_interfaces::msg::MotorsControllerOutput>::SharedPtr motors_controller_data_subscriber;
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_data_subscriber;
		rclcpp::Subscription<minirys_interfaces::msg::MinirysInput>::SharedPtr minirys_subscriber;

		rclcpp::Client<minirys_interfaces::srv::SetOdometryPosition>::SharedPtr set_odometry_position_client;
		rclcpp::Client<minirys_interfaces::srv::SetOdometryPosition>::SharedRequest set_odometry_position_request;

		void imuDataCallback(const minirys_interfaces::msg::ImuOutput::SharedPtr msg);
		void temperatureCallback(const sensor_msgs::msg::Temperature::SharedPtr msg);
		void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
		void tofDataCallback(const minirys_interfaces::msg::TofOutput::SharedPtr msg);
		void motorsControllerDataCallback(const minirys_interfaces::msg::MotorsControllerOutput::SharedPtr msg);
		void odometryDataCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
		void minirysCallback(const minirys_interfaces::msg::MinirysInput::SharedPtr msg);
};