#include "PID.h"
#include <cmath>

using namespace std;


PID::PID() {
  d_error = 0;
  p_error = 0;
  i_error = 0;
  abs_error = 0;
  n = 0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void PID::UpdateError(double cte) {
  if (n > 0)
    d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

  abs_error += abs(cte);
  n++;
}

double PID::Control() {
  return -Kp * p_error - Kd * d_error - Ki * i_error;
}

double PID::TotalError() {
  return abs_error;
}

double PID::AvgError() {
  return abs_error / n;
}

