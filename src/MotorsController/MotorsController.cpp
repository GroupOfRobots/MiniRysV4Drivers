#include "MotorsController/MotorsController.hpp"
#include <cmath>
#include <stdexcept>
#include <iostream>

MotorsController::MotorsController() {
	this->board = new Motors( BCM2835_SPI_CS0, GPIO_RESET_OUT);
	this->board->setUp();
	this->board->resetPosition();
	this->balancing = false;

	this->pidSpeedRegulatorEnabled = true;
	this->pidSpeedKp = 0;
	this->pidSpeedInvTi = 0;
	this->pidSpeedTd = 0;
	this->pidSpeedPreviousError1 = 0;
	this->pidSpeedPreviousError2 = 0;
	this->pidAngleKp = 0;
	this->pidAngleInvTi = 0;
	this->pidAngleTd = 0;
	this->pidAnglePreviousError1 = 0;
	this->pidAnglePreviousError2 = 0;
	this->pidPreviousTargetAngle = 0;

	this->motorsEnabled = false;
	this->motorSpeedLeft = 0;
	this->motorSpeedRight = 0;
	this->invertLeftSpeed = false;
	this->invertRightSpeed = false;
	this->maxAcceleration = 400.0;
	this->maxSpeed = 400.0;

	this->disableMotors();
}

MotorsController::~MotorsController() {
	this->disableMotors();
	this->board->stop();
	delete this->board;
}

void clipValue(float & value, float max) {
	if (value > max) {
		value = max;
	} else if (value < -max) {
		value = -max;
	}
}

void MotorsController::setInvertSpeed(const bool left, const bool right) {
	this->invertLeftSpeed = left;
	this->invertRightSpeed = right;
}

void MotorsController::setBalancing(bool value) {
	this->balancing = value;
}

void MotorsController::setPIDSpeedRegulatorEnabled(bool enabled) {
	this->pidSpeedRegulatorEnabled = enabled;
}

void MotorsController::setPIDParameters(float speedKp, float speedInvTi, float speedTd, float angleKp, float angleInvTi, float angleTd) {
	this->pidSpeedKp = speedKp;
	this->pidSpeedInvTi = speedInvTi;
	this->pidSpeedTd = speedTd;
	this->pidAngleKp = angleKp;
	this->pidAngleInvTi = angleInvTi;
	this->pidAngleTd = angleTd;
}

void MotorsController::zeroPIDRegulator(){
	this->pidSpeedPreviousError1 = 0;
	this->pidSpeedPreviousError2 = 0;
	this->pidAnglePreviousError1 = 0;
	this->pidAnglePreviousError2 = 0;
	this->pidPreviousTargetAngle = 0;
}

bool MotorsController::getPIDSpeedRegulatorEnabled() {
	return this->pidSpeedRegulatorEnabled;
}

void MotorsController::getPIDParameters(float & speedKp, float & speedInvTi, float & speedTd, float & angleKp, float & angleInvTi, float & angleTd) {
	speedKp = this->pidSpeedKp;
	speedInvTi = this->pidSpeedInvTi;
	speedTd = this->pidSpeedTd;
	angleKp = this->pidAngleKp;
	angleInvTi = this->pidAngleInvTi;
	angleTd = this->pidAngleTd;
}

void MotorsController::getPIDPreviousTargetAngle(float & angle){
	angle = this->pidPreviousTargetAngle;
}

void MotorsController::setMaxAcceleration(float acceleration){
	this->maxAcceleration = acceleration;
}

void MotorsController::setMaxSpeed(float speed){
	this->maxSpeed = speed;
	this->board->setMaxSpeedForBoth(speed);
}

void MotorsController::setMicrostep(int step){
	if (step != 1 && (step == 0 || step % 2 || step > 128)) {
		throw(std::domain_error("Bad microstep value! Allowed ones are powers of 2 from 1 to 128"));
	}
	this-> microstep = step;
	switch(step){
		case 1:
			this->board->setMicrostep(0x00);
			break;
		case 2:
			this->board->setMicrostep(0x01);
			break;
		case 4:
			this->board->setMicrostep(0x02);
			break;
		case 8:
			this->board->setMicrostep(0x03);
			break;
		case 16:
			this->board->setMicrostep(0x04);
			break;
		case 32:
			this->board->setMicrostep(0x05);
			break;
		case 64:
			this->board->setMicrostep(0x06);
			break;
		case 128:
			this->board->setMicrostep(0x07);
			break;
		default:
			break;
	}
}

void MotorsController::calculateSpeeds(float angle, float rotationX, float throttle, float rotation, float &speedLeftNew, float &speedRightNew, float loopTime) {
	clipValue(throttle, this->maxSpeed);
	clipValue(rotation, this->maxSpeed);

	if (!this->balancing) {
		speedLeftNew = throttle + rotation;
		speedRightNew = throttle - rotation;
		clipValue(speedLeftNew, this->maxSpeed);
		clipValue(speedRightNew, this->maxSpeed);
		return;
	}

	this->calculateSpeedsPID(angle, rotationX, throttle, rotation, speedLeftNew, speedRightNew, loopTime);
}

void MotorsController::calculateSpeedsPID(float angle, float rotationX, float throttle, float rotation, float &speedLeftNew, float &speedRightNew, float loopTime) {
	float targetAngle = 0.0f;
	float speedError = 0.0f;
	float speed = (this->getMotorSpeedLeft() + this->getMotorSpeedRight())/2;

	if (this->pidSpeedRegulatorEnabled) {
		speedError = speed - rotationX*RAD_TO_DEG - throttle;

		float speedFactor0 = this->pidSpeedKp * (1 + this->pidSpeedInvTi * loopTime / 2 + this->pidSpeedTd / loopTime);
		float speedFactor1 = this->pidSpeedKp * (this->pidSpeedInvTi * loopTime / 2 - 2 * this->pidSpeedTd / loopTime - 1);
		float speedFactor2 = this->pidSpeedKp * this->pidSpeedTd / loopTime;

		targetAngle = speedFactor0 * speedError + speedFactor1 * this->pidSpeedPreviousError1 + speedFactor2 * this->pidSpeedPreviousError2 + this->pidPreviousTargetAngle;
		clipValue(targetAngle, 0.25);
	}
	this->pidSpeedPreviousError2 = this->pidSpeedPreviousError1;
	this->pidSpeedPreviousError1 = speedError;
	this->pidPreviousTargetAngle = targetAngle;

	float angleError = targetAngle - angle;

	float angleFactor0 = this->pidAngleKp * (1 + this->pidAngleInvTi * loopTime / 2 + this->pidAngleTd / loopTime);
	float angleFactor1 = this->pidAngleKp * (this->pidAngleInvTi * loopTime / 2 - 2 * this->pidAngleTd / loopTime - 1);
	float angleFactor2 = this->pidAngleKp * this->pidAngleTd / loopTime;

	float output = angleFactor0 * angleError + angleFactor1 * this->pidAnglePreviousError1 + angleFactor2 * this->pidAnglePreviousError2 + speed;
	clipValue(output, this->maxSpeed);
	clipValue(rotation, this->maxSpeed);

	speedLeftNew = output + rotation;
	speedRightNew = output - rotation;
	clipValue(speedLeftNew, this->maxSpeed);
	clipValue(speedRightNew, this->maxSpeed);

	this->pidAnglePreviousError2 = this->pidAnglePreviousError1;
	this->pidAnglePreviousError1 = angleError;
}

void MotorsController::enableMotors() {
	this->motorsEnabled = true;
}

void MotorsController::disableMotors() {
	this->motorsEnabled = false;
	this->motorSpeedLeft = 0;
	this->motorSpeedRight = 0;
	this->setMotorSpeeds(0.0, 0.0, true);
}

void MotorsController::setMotorSpeeds(float speedLeft, float speedRight, bool ignoreAcceleration) {
	// Clip speed values
	clipValue(speedLeft, this->maxSpeed);
	clipValue(speedRight, this->maxSpeed);

	// If needed, invert the speeds
	if (this->invertLeftSpeed) {
		speedLeft = -speedLeft;
	}
	if (this->invertRightSpeed) {
		speedRight = -speedRight;
	}

	// Clip speeds according to acceleration limits
	if (ignoreAcceleration) {
		this->motorSpeedLeft = speedLeft;
		this->motorSpeedRight = speedRight;
	} else {
		if (speedLeft > (this->motorSpeedLeft + this->maxAcceleration)) {
			this->motorSpeedLeft = this->motorSpeedLeft + this->maxAcceleration;
		} else if (speedLeft < (this->motorSpeedLeft - this->maxAcceleration)) {
			this->motorSpeedLeft = this->motorSpeedLeft - this->maxAcceleration;
		} else {
			this->motorSpeedLeft = speedLeft;
		}
		if (speedRight > (this->motorSpeedRight + this->maxAcceleration)) {
			this->motorSpeedRight = this->motorSpeedRight + this->maxAcceleration;
		} else if (speedRight < (this->motorSpeedRight - this->maxAcceleration)) {
			this->motorSpeedRight = this->motorSpeedRight - this->maxAcceleration;
		} else {
			this->motorSpeedRight = speedRight;
		}
	}

	this->board->setSpeed(this->motorSpeedLeft, this->motorSpeedRight);
}

float MotorsController::getMotorSpeedLeft() const {
	float inversionMultiplier = this->invertLeftSpeed ? -1 : 1;
	return this->motorSpeedLeft * inversionMultiplier;
}

float MotorsController::getMotorSpeedRight() const {
	float inversionMultiplier = this->invertRightSpeed ? -1 : 1;
	return this->motorSpeedRight * inversionMultiplier;
}
