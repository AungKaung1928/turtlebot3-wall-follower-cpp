#include "wall_following_cpp_project/pid_controller.hpp"

namespace wall_following
{

PidController::PidController(double kp, double kd, double dt, double derivative_alpha)
: kp_(kp), kd_(kd), dt_(dt), alpha_(derivative_alpha)
{
}

double PidController::update(double error)
{
  const double raw_derivative = (error - prev_error_) / dt_;
  filtered_derivative_ = (1.0 - alpha_) * filtered_derivative_ + alpha_ * raw_derivative;
  prev_error_ = error;
  return kp_ * error + kd_ * filtered_derivative_;
}

void PidController::reset()
{
  prev_error_ = 0.0;
  filtered_derivative_ = 0.0;
}

}  // namespace wall_following
