#pragma once

namespace wall_following
{

/// PD controller with a low-pass filtered derivative. The raw derivative divides by dt, which
/// amplifies scan noise 1/dt times, so it is EMA-filtered before the gain is applied.
class PidController
{
public:
  PidController(double kp, double kd, double dt, double derivative_alpha = 0.3);

  double update(double error);
  void reset();

private:
  double kp_;
  double kd_;
  double dt_;
  double alpha_;
  double prev_error_{0.0};
  double filtered_derivative_{0.0};
};

}  // namespace wall_following
