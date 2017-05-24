#include "PID.h"

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
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void PID::UpdateError(double cte) {
  i_error = cte - p_error;
  p_error = cte;
  d_error += cte;

  steer_value = - Kp * p_error - Ki * i_error - Kd * d_error;
  throttle = 0.3f;
}

double PID::TotalError() {

}
