#include "PID.h"
#include <math.h>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
  steer_value = 0.0f;
  throttle = 0.3f;
  p_error = 0.0f;
  i_error = 0.0f;
  d_error = 0.0f;
  total_error = 0.0f;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->steer_value = 0.0f;
  this->total_error = 0.0f;
}

void PID::UpdateError(double cte, double speed, double angle) {

  speed = fmax(0.1, speed);

  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

  // Define loss (total_error) as sum of cte square devided by speed
  //   tell the system to stay close to center line as close as
  //   possible also do not stuck to some really low speed movement
  total_error += cte * cte / speed;

  double new_steer = - Kp * p_error - Ki * i_error - Kd * d_error;
  steer_value = new_steer;

  steer_value = fmax(-1.0, steer_value);
  steer_value = fmin(1.0, steer_value);

  throttle = 0.40f;
}

double PID::TotalError() {
  return total_error;
}
