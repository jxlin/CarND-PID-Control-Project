#include "PID.h"
#include <math.h>

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

void PID::UpdateError(double cte) {

  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
  total_error += cte * cte;  // sum of square error

  steer_value = - Kp * p_error - Ki * i_error - Kd * d_error;
  steer_value = fmax(-1.0, steer_value);
  steer_value = fmin(1.0, steer_value);

  throttle = 0.45f;
}

double PID::TotalError() {
  return total_error;
}
