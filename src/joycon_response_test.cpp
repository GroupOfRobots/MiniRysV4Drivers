#include <stdio.h>
#include <fstream>
#include <csignal>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#include "bcm/bcm2835.h"

#define JOY_DEV "/dev/input/js0"

bool endProcess = false;

void sigintHandler(int signum) {
	if (signum == SIGINT) {
		endProcess = true;
	}
}

int main(void)
{
	signal(SIGINT, sigintHandler);

	int joy_fd, *axis=NULL, *axisPast=NULL, num_of_axis=0, num_of_buttons=0;
	bool *button=NULL, *buttonPast=NULL;
	char name_of_joystick[80];//*button=NULL,
	struct js_event js;

	if( ( joy_fd = open( JOY_DEV , O_RDONLY)) == -1 )
	{
		printf( "Couldn't open joystick\n" );
		return 0;
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

	printf("Joystick detected: %s\n\t%d axis\n\t%d buttons\n\n"
	, name_of_joystick
	, num_of_axis
	, num_of_buttons );

	fcntl( joy_fd, F_SETFL, O_NONBLOCK ); /* use non-blocking mode */

	while( !endProcess )  /* infinite loop */
	{
		/* read the joystick state */
		read(joy_fd, &js, sizeof(struct js_event));

		/* see what to do with the event */
		switch (js.type & ~JS_EVENT_INIT)
		{
			case JS_EVENT_AXIS:
				axis   [ js.number ] = js.value;
				if (axis [ js.number ] != axisPast[js.number]) printf("Axis %d:\t%d\n", js.number, axis[js.number]);
				axisPast[js.number] = axis[js.number];
				break;
			case JS_EVENT_BUTTON:
				button [ js.number ] = js.value;
				if (button [ js.number ] != buttonPast[js.number]) printf("Button %d:\t%d\n", js.number, button[js.number]);
				buttonPast[js.number] = button[js.number];
				break;
		}
		delay(10);
	}

	close( joy_fd ); /* too bad we never get here */
	return 0;
}