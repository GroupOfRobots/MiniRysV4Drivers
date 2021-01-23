#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#include "rclcpp/rclcpp.hpp"
#include "../data_structures.h"
#include "../FrequencyCounter/FrequencyCounter.hpp"

#define JOY_DEV "/dev/input/js0"

class JoyconReceiverNode : public rclcpp::Node{
	public:
		JoyconReceiverNode(robot_control_data& structure);
		~JoyconReceiverNode();

	private:
		int joy_fd, *axis, *axisPast, num_of_axis, num_of_buttons;
		bool *button, *buttonPast;
		char name_of_joystick[80];
		struct js_event js;

		int forwardAxis, rotationAxis, forwardSpeedFactor, rotationSpeedFactor;
		bool forwardAxisInverted, rotationAxisInverted;

		int standUpButton, layDownButton, printStatusButton, printLocationButton;

		robot_control_data *dataStructure; 
		FrequencyCounter counter;

		rclcpp::TimerBase::SharedPtr get_joycon_state_timer;

		void get_joycon_state();
};