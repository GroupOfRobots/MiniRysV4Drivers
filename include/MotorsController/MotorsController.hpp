#ifndef _MOTORS_CONTROLLER_HPP
#define _MOTORS_CONTROLLER_HPP

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <mutex>
#include "../l6470/motors.h"
#include "../bcm/bcm2835.h"

#define DEG_TO_RAD 0.017453f
#define RAD_TO_DEG 57.295779f

void clipValue(float & value, float max);

struct motor_status {
	bool status[13];
	motor_status(){
		for (int i; i < 13; i++) status[i] = false;
	};
};

class MotorsController {
	private:
		Motors *board;

		bool balancing;
		bool standingUpPhase;
		int standingUpDir;
		int standingUpCounter;

		bool pidSpeedRegulatorEnabled;
		float pidSpeedKp;
		float pidSpeedInvTi;
		float pidSpeedTd;
		float pidSpeedPreviousError1;
		float pidSpeedPreviousError2;
		float pidAngleKp;
		float pidAngleInvTi;
		float pidAngleTd;
		float pidAnglePreviousError1;
		float pidAnglePreviousError2;
		float pidPreviousTargetAngle;

		bool motorsEnabled;
		float motorSpeedLeft;
		float motorSpeedRight;
		bool invertLeftSpeed;
		bool invertRightSpeed;
		int microstep;
		float maxAcceleration;
		float maxSpeed;

		void calculateSpeedsPID(float angle, float rotationX, float throttle, float rotation, float &speedLeftNew, float &speedRightNew, float loopTime);
	public:
		MotorsController();
		~MotorsController();

		void setInvertSpeed(const bool left, const bool right);

		void setBalancing(bool value);
		bool getBalancing();
		void standUp(float angle, float &speedLeftNew, float &speedRightNew);
		void resetStandUp();

		void setPIDSpeedRegulatorEnabled(bool enabled);
		void setPIDParameters(float speedKp, float speedInvTi, float speedTd, float angleKp, float angleInvTi, float angleTd);
		void zeroPIDRegulator();
		bool getPIDSpeedRegulatorEnabled();
		void getPIDParameters(float & speedKp, float & speedInvTi, float & speedTd, float & angleKp, float & angleInvTi, float & angleTd);
		void getPIDPreviousTargetAngle(float & angle);
		void setMaxAcceleration(float acceleration);
		void setMaxSpeed(float speed);
		void setMicrostep(int m);

		void calculateSpeeds(float angle, float rotationX, float throttle, float rotation, float &speedLeftNew, float &speedRightNew, float loopTime);

		void enableMotors();
		void disableMotors();

		void setMotorSpeeds(float speedLeft, float speedRight, bool ignoreAcceleration = false);
		float getMotorSpeedLeft() const;
		float getMotorSpeedRight() const;

		void getMotorsStatusRegisters(long &motor0, long &motor1);
		motor_status getMotorStatusLeft();
		motor_status getMotorStatusRight();
		void getMotorsSpeedConfiguration(float &max, float &min, float &acc, float &dec);
};

#endif
