#include "controller.h"

// PD controller implementation(Proportional, derivative). DT is in miliseconds
float Controller::stabilityPDControl(float DT, float input, float setPoint,  float Kp, float Kd)
{
  float error;
  float output;

  error = setPoint - input;

  // Kd is implemented in two parts
  //    The biggest one using only the input (sensor) part not the SetPoint input-input(t-2)
  //    And the second using the setpoint to make it a bit more agressive   setPoint-setPoint(t-1)
  output = Kp * error + (Kd * (setPoint - setPointOld) - Kd * (input - PID_errorOld2)) / DT;
  PID_errorOld2 = PID_errorOld;
  PID_errorOld = input;  // error for Kd is only the input component
  setPointOld = setPoint;
  return (output);
}

// PI controller implementation (Proportional, integral). DT is in miliseconds
float Controller::speedPIControl(float DT, float input, float setPoint,  float Kp, float Ki)
{
  float error;
  float output;

  error = setPoint - input;
  if(error>ITERM_MAX_ERROR)error = ITERM_MAX_ERROR;
  if(error<-ITERM_MAX_ERROR)error = -ITERM_MAX_ERROR;

  PID_errorSum += error;
  if(PID_errorSum>ITERM_MAX)PID_errorSum = ITERM_MAX;
  if(PID_errorSum<-ITERM_MAX)PID_errorSum = -ITERM_MAX;


  output = Kp * error + Ki * PID_errorSum * DT * 0.001; // DT is in miliseconds...
  return (output);
}

void Controller::calculate_speed(float actualangle,float actualleftspeed, float actualrightspeed, int steering, int throttle, float &speedleft, float &speedright){


  timerStop(); //timer value;
  dt = timerValue()*1000;
  timerStart();

  angle_adjusted_Old = angle_adjusted;
  angle_adjusted = actualangle;


  actual_robot_speed_Old = actual_robot_speed;
  actual_robot_speed = (actualleftspeed + actualrightspeed) / 2; // Positive: forward

  float angular_velocity = (angle_adjusted - angle_adjusted_Old) * 90.0; // 90 is an empirical extracted factor to adjust for real units
  float estimated_speed = -actual_robot_speed_Old - angular_velocity;     // We use robot_speed(t-1) to compensate the delay
  estimated_speed_filtered = estimated_speed_filtered * 0.95 + estimated_speed * 0.05;  // low pass filter on estimated speed

    // SPEED CONTROL: This is a PI controller.
    //    input:user throttle, variable: estimated robot speed, output: target robot angle to get the desired speed
  target_angle = speedPIControl(dt, estimated_speed_filtered, throttle, Kp_thr, Ki_thr);
  if(target_angle>max_target_angle)target_angle = max_target_angle;
  if(target_angle<-max_target_angle)target_angle = -max_target_angle;

    // Stability control: This is a PD controller.
    //    input: robot target angle(from SPEED CONTROL), variable: robot angle, output: Motor speed
    //    We integrate the output (sumatory), so the output is really the motor acceleration, not motor speed.
  control_output = stabilityPDControl(dt, angle_adjusted, target_angle, Kp, Kd);
  if(control_output>MAX_CONTROL_OUTPUT)control_output = MAX_CONTROL_OUTPUT;
  if(control_output<-MAX_CONTROL_OUTPUT)control_output = -MAX_CONTROL_OUTPUT;

    // The steering part from the user is injected directly on the output
  speedleft = control_output + steering;
  speedright = control_output - steering;

}

void Controller::timerStart(){
	timer_old = std::chrono::high_resolution_clock::now();
}

void Controller::timerStop(){
	timer_value = std::chrono::high_resolution_clock::now();
}

float Controller::timerValue(){
	std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(timer_value - timer_old);

	return time_span.count();
}
