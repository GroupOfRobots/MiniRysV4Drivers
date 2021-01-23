#include "nodes/JoyconReceiverNode.hpp"

JoyconReceiverNode::JoyconReceiverNode(robot_control_data& structure): Node("joycon_receiver") {
	dataStructure = &structure;
	//initialize joycon
	if( ( joy_fd = open( JOY_DEV , O_RDONLY)) == -1 )
	{
		RCLCPP_ERROR(this->get_logger(), "Couldn't open joystick" );
		// endProcess = true;
		return;
	}

	ioctl( joy_fd, JSIOCGAXES, &num_of_axis );
	ioctl( joy_fd, JSIOCGBUTTONS, &num_of_buttons );
	ioctl( joy_fd, JSIOCGNAME(80), &name_of_joystick );

	axis = (int *) calloc( num_of_axis, sizeof( int ) );
	axisPast = (int *) calloc( num_of_axis, sizeof( int ) );
	for(int i = 0; i < num_of_axis; i++){
		axis[i] = 0;
		axisPast[i] = 0;
	}
	button = (bool *) calloc( num_of_buttons, sizeof( bool ) );
	buttonPast = (bool *) calloc( num_of_buttons, sizeof( bool ) );
	for(int i = 0; i < num_of_buttons; i++){
		button[i] = false;
		buttonPast[i] = false;
	}

	RCLCPP_INFO(this->get_logger(),"Joystick detected: %s\n\t%d axis\n\t%d buttons\n\n"
	, name_of_joystick
	, num_of_axis
	, num_of_buttons );

	fcntl( joy_fd, F_SETFL, O_NONBLOCK ); /* use non-blocking mode */

	// Get axis parameters
	this->declare_parameter("forwardAxis", rclcpp::ParameterValue(1));
	forwardAxis = this->get_parameter("forwardAxis").get_value<int>();
	this->declare_parameter("rotationAxis", rclcpp::ParameterValue(2));
	rotationAxis = this->get_parameter("rotationAxis").get_value<int>();
	this->declare_parameter("forwardSpeedFactor", rclcpp::ParameterValue(80));
	forwardSpeedFactor = this->get_parameter("forwardSpeedFactor").get_value<int>();
	this->declare_parameter("rotationSpeedFactor", rclcpp::ParameterValue(200));
	rotationSpeedFactor = this->get_parameter("rotationSpeedFactor").get_value<int>();
	this->declare_parameter("forwardAxisInverted", rclcpp::ParameterValue(true));
	forwardAxisInverted = this->get_parameter("forwardAxisInverted").get_value<bool>();
	this->declare_parameter("rotationAxisInverted", rclcpp::ParameterValue(false));
	rotationAxisInverted = this->get_parameter("rotationAxisInverted").get_value<bool>();

	// Get button parameters
	this->declare_parameter("standUpButton", rclcpp::ParameterValue(1));
	standUpButton = this->get_parameter("standUpButton").get_value<int>();
	this->declare_parameter("layDownButton", rclcpp::ParameterValue(3));
	layDownButton = this->get_parameter("layDownButton").get_value<int>();
	this->declare_parameter("printStatusButton", rclcpp::ParameterValue(2));
	printStatusButton = this->get_parameter("printStatusButton").get_value<int>();
	this->declare_parameter("printLocationButton", rclcpp::ParameterValue(0));
	printLocationButton = this->get_parameter("printLocationButton").get_value<int>();

	this->declare_parameter("period", rclcpp::ParameterValue(10));
	get_joycon_state_timer = this->create_wall_timer(
	std::chrono::milliseconds(this->get_parameter("period").get_value<int>()), std::bind(&JoyconReceiverNode::get_joycon_state, this));
	RCLCPP_INFO(this->get_logger(), "Joycon receiver initialized.");
}

JoyconReceiverNode::~JoyconReceiverNode(){
	close( joy_fd );
}

void JoyconReceiverNode::get_joycon_state() {
	counter.count();
	/* read the joystick state */
	/* see what to do with the event */
	while(read(joy_fd, &js, sizeof(struct js_event)) == sizeof(struct js_event)){
		switch (js.type & ~JS_EVENT_INIT)
		{
			case JS_EVENT_AXIS:
				axis   [ js.number ] = js.value;
				// if (axis [ js.number ] != axisPast[js.number]) RCLCPP_INFO(this->get_logger(),"Axis %d:\t%d", js.number, axis[js.number]);
				axisPast[js.number] = axis[js.number];
				break;
			case JS_EVENT_BUTTON:
				button [ js.number ] = js.value;
				if (button [ js.number ] != buttonPast[js.number]) RCLCPP_INFO(this->get_logger(),"Button %d:\t%d", js.number, button[js.number]);
				buttonPast[js.number] = button[js.number];
				break;
		}
	}

	dataStructure->robot_control_data_access.lock();

	if(!forwardAxisInverted){
		dataStructure->forwardSpeed = axis[forwardAxis]/forwardSpeedFactor;
	} else {
		dataStructure->forwardSpeed = -axis[forwardAxis]/forwardSpeedFactor;
	}

	if(!rotationAxisInverted){
		dataStructure->rotationSpeed = axis[rotationAxis]/rotationSpeedFactor;
	} else {
		dataStructure->rotationSpeed = -axis[rotationAxis]/rotationSpeedFactor;
	}

	if (button[standUpButton] == 1) dataStructure->enableBalancing = true;
	if (button[layDownButton] == 1) dataStructure->enableBalancing = false;
	dataStructure->printMotorStatus = button[printStatusButton] == 1 ? true : false;
	dataStructure->printRobotLocation = button[printLocationButton] == 1 ? true : false;

	dataStructure->robot_control_data_access.unlock();
}