#include "wall_following_cpp_project/wall_follower_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace wall_following
{

using geometry_msgs::msg::Twist;

WallFollowerController::WallFollowerController(const rclcpp::NodeOptions & options)
: Node("wall_follower_controller", options)
{
  declareParameters();
  loadParameters();
  detector_ = std::make_unique<WallDetector>(p_.beam_spread_deg);
  pid_ = std::make_unique<PidController>(p_.kp, p_.kd, 1.0 / p_.control_frequency);

  cmd_pub_ = create_publisher<Twist>("/cmd_vel", 10);
  state_pub_ = create_publisher<std_msgs::msg::String>("/wall_follower/state", 10);
  // SensorDataQoS (best effort) receives from both the sim bridge and the real LDS driver.
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(),
    std::bind(&WallFollowerController::laserCallback, this, std::placeholders::_1));
  const auto period = std::chrono::duration<double>(1.0 / p_.control_frequency);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&WallFollowerController::controlLoop, this));

  RCLCPP_INFO(get_logger(), "Wall follower ready: target %.2f m, Kp=%.2f Kd=%.2f, %.0f Hz",
    p_.desired_distance, p_.kp, p_.kd, p_.control_frequency);
}

void WallFollowerController::declareParameters()
{
  declare_parameter<double>("desired_distance", 0.6);
  declare_parameter<double>("forward_speed", 0.20);
  declare_parameter<double>("search_speed", 0.16);
  declare_parameter<double>("max_angular_speed", 0.6);
  declare_parameter<double>("kp", 1.8);
  declare_parameter<double>("kd", 0.7);
  declare_parameter<double>("lookahead_distance", 0.3);
  declare_parameter<double>("beam_spread_deg", 40.0);
  declare_parameter<double>("emergency_stop_distance", 0.55);
  declare_parameter<double>("slow_down_distance", 0.8);
  declare_parameter<double>("wall_min_distance", 0.45);
  declare_parameter<double>("wall_lost_distance", 1.5);
  declare_parameter<double>("side_clearance", 0.4);
  declare_parameter<int>("search_period_cycles", 60);
  declare_parameter<int>("stuck_threshold_cycles", 3);
  declare_parameter<double>("control_frequency", 20.0);
}

double WallFollowerController::paramInRange(const std::string & name, double lo, double hi)
{
  const double v = get_parameter(name).as_double();
  if (v < lo || v > hi) {
    RCLCPP_ERROR(get_logger(), "%s=%.3f outside [%.2f, %.2f]; using midpoint",
      name.c_str(), v, lo, hi);
    return 0.5 * (lo + hi);
  }
  return v;
}

void WallFollowerController::loadParameters()
{
  p_.desired_distance = paramInRange("desired_distance", 0.3, 1.5);
  p_.forward_speed = paramInRange("forward_speed", 0.05, 0.5);
  p_.search_speed = paramInRange("search_speed", 0.05, 0.3);
  p_.max_angular_speed = paramInRange("max_angular_speed", 0.1, 1.5);
  p_.kp = paramInRange("kp", 0.1, 5.0);
  p_.kd = paramInRange("kd", 0.0, 2.0);
  p_.lookahead = paramInRange("lookahead_distance", 0.1, 1.0);
  p_.beam_spread_deg = paramInRange("beam_spread_deg", 20.0, 70.0);
  p_.emergency_stop = paramInRange("emergency_stop_distance", 0.2, 1.0);
  p_.slow_down = paramInRange("slow_down_distance", 0.3, 1.5);
  p_.wall_min = paramInRange("wall_min_distance", 0.2, 0.8);
  p_.wall_lost = paramInRange("wall_lost_distance", 0.8, 3.0);
  p_.side_clearance = paramInRange("side_clearance", 0.2, 0.8);
  p_.search_period = get_parameter("search_period_cycles").as_int();
  p_.stuck_threshold = get_parameter("stuck_threshold_cycles").as_int();
  p_.control_frequency = paramInRange("control_frequency", 5.0, 100.0);
  if (p_.emergency_stop >= p_.slow_down) {
    RCLCPP_WARN(get_logger(), "emergency_stop >= slow_down; using 0.7 x slow_down");
    p_.emergency_stop = 0.7 * p_.slow_down;
  }
}

void WallFollowerController::laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  scan_ = std::move(msg);
}

const char * WallFollowerController::name(State s)
{
  switch (s) {
    case State::SEARCHING: return "SEARCHING";
    case State::FOLLOWING: return "FOLLOWING";
    default: return "AVOIDING";
  }
}

void WallFollowerController::setState(State next, const std::string & reason)
{
  if (next != state_) {
    RCLCPP_INFO(get_logger(), "%s -> %s: %s", name(state_), name(next), reason.c_str());
  }
  state_ = next;
  state_counter_ = 0;
  pid_->reset();
}

std::pair<bool, double> WallFollowerController::detectCollisionThreat(double margin) const
{
  const auto & s = *scan_;
  const double stop = p_.emergency_stop * margin;
  const double side = p_.side_clearance * margin;
  const double front_center = detector_->minInArc(s, -20, 20, 2);
  const double front_left = detector_->minInArc(s, 20, 50, 3);
  const double front_right = detector_->minInArc(s, -50, -20, 3);
  const double wide_left = detector_->minInArc(s, 50, 70, 5);
  const double wide_right = detector_->minInArc(s, -70, -50, 5);
  double right_side = detector_->minInArc(s, -90, -60, 3);
  double left_side = detector_->minInArc(s, 60, 90, 3);
  // The followed wall is expected close; followWall() guards it with wall_min instead.
  if (state_ == State::FOLLOWING) {
    (wall_sign_ < 0 ? right_side : left_side) = std::numeric_limits<double>::infinity();
  }
  const bool threat = front_center < stop || front_left < stop || front_right < stop ||
    right_side < side || left_side < side || wide_left < side || wide_right < side;
  return {threat, std::min({front_center, front_left, front_right})};
}

Twist WallFollowerController::searchForWall()
{
  const auto & s = *scan_;
  const double right = detector_->rangeAt(s, -90.0, true);
  const double left = detector_->rangeAt(s, 90.0, true);
  if (right < p_.wall_lost || left < p_.wall_lost) {
    wall_sign_ = right < left ? -1 : 1;
    char msg[64];
    std::snprintf(msg, sizeof(msg), "wall on %s at %.2f m",
      wall_sign_ < 0 ? "RIGHT" : "LEFT", std::min(right, left));
    setState(State::FOLLOWING, msg);
    return Twist();
  }
  Twist cmd;
  const double front = detector_->minInArc(s, -25, 25, 2);
  cmd.linear.x = front > p_.slow_down ? p_.search_speed : 0.4 * p_.search_speed;
  if (++state_counter_ > p_.search_period) {
    search_direction_ = -search_direction_;
    state_counter_ = 0;
  }
  cmd.angular.z = 0.25 * search_direction_;
  return cmd;
}

Twist WallFollowerController::followWall()
{
  const auto & s = *scan_;
  Twist cmd;
  const int lo = wall_sign_ < 0 ? -135 : 45;
  const double arc_min = detector_->minInArc(s, lo, lo + 90, 3);
  const auto wall = detector_->estimate(s, wall_sign_);

  // Wall ended beside us (outside corner): both beams look past it while the rear-side arc
  // still has it. Arc around it at the standoff radius instead of entering the search sweep.
  if (!wall && arc_min <= p_.wall_lost) {
    cmd.linear.x = p_.search_speed;
    cmd.angular.z = wall_sign_ * p_.search_speed / p_.desired_distance;
    return cmd;
  }
  const double wall_dist = wall ? wall->distance : arc_min;
  const double alpha = wall ? wall->angle : 0.0;

  if (std::min(wall_dist, arc_min) > p_.wall_lost) {
    search_direction_ = wall_sign_;  // curve toward the side the wall was on
    char msg[48];
    std::snprintf(msg, sizeof(msg), "wall lost at %.2f m", wall_dist);
    setState(State::SEARCHING, msg);
    return Twist();
  }
  if (wall_dist < p_.wall_min) {
    cmd.linear.x = 0.08;
    cmd.angular.z = -wall_sign_ * 0.6;
    return cmd;
  }

  // Steer on the distance predicted lookahead metres ahead: a heading that converges on the
  // wall shrinks the error before the wall arrives, which is what keeps the lock through turns.
  const double predicted = wall_dist - p_.lookahead * std::sin(alpha);
  const double error = predicted - p_.desired_distance;  // > 0: too far
  const double omega = std::clamp(pid_->update(error), -p_.max_angular_speed, p_.max_angular_speed);
  cmd.angular.z = wall_sign_ * omega;

  const double front = detector_->minInArc(s, -30, 30, 2);
  double speed = p_.forward_speed;
  if (front < p_.slow_down) {
    const double f = (front - p_.emergency_stop) / (p_.slow_down - p_.emergency_stop);
    speed *= std::clamp(f, 0.2, 1.0);
  }
  if (std::fabs(cmd.angular.z) > 0.3) {
    speed *= 1.0 - 0.7 * std::fabs(cmd.angular.z) / p_.max_angular_speed;
  }
  cmd.linear.x = std::max(0.05, speed);
  return cmd;
}

int WallFollowerController::pickEscapeDirection() const
{
  if (state_ == State::FOLLOWING) {
    return -wall_sign_;  // away from the followed wall; into it is never the escape
  }
  const auto & s = *scan_;
  const double left = detector_->minInArc(s, 45, 135, 5);
  const double right = detector_->minInArc(s, -135, -45, 5);
  return left >= right ? 1 : -1;
}

Twist WallFollowerController::avoidCollision()
{
  Twist cmd;
  ++state_counter_;
  // Rotating in place cannot help when every heading is blocked (wedged in a corner):
  // back off once the turn has clearly failed, but only if there is room behind.
  if (state_counter_ > p_.stuck_threshold * 20 &&
    detector_->minInArc(*scan_, 150, 210, 5) > 0.30)
  {
    cmd.linear.x = -0.06;
  }
  cmd.angular.z = 0.8 * escape_dir_;
  return cmd;
}

void WallFollowerController::controlLoop()
{
  if (!scan_) {
    return;
  }
  // Clearing needs a wider margin than tripping, otherwise the state chatters on the threshold.
  const auto [threat, front] = detectCollisionThreat(state_ == State::AVOIDING ? 1.3 : 1.0);
  Twist cmd;
  if (threat) {
    if (state_ != State::AVOIDING) {
      resume_state_ = state_;
      escape_dir_ = pickEscapeDirection();
      char msg[48];
      std::snprintf(msg, sizeof(msg), "front %.2f m", front);
      setState(State::AVOIDING, msg);
    }
    cmd = avoidCollision();
  } else {
    if (state_ == State::AVOIDING) {
      setState(resume_state_, "clear");  // keep the wall lock through a corner
    }
    cmd = state_ == State::SEARCHING ? searchForWall() : followWall();
  }
  // Lower bound is negative so avoidCollision() can back out of a corner.
  cmd.linear.x = std::clamp(cmd.linear.x, -0.08, p_.forward_speed);
  cmd.angular.z = std::clamp(cmd.angular.z, -p_.max_angular_speed, p_.max_angular_speed);
  // Emergency brake for something already inside the stop distance.
  if (detector_->minInArc(*scan_, -15, 15, 1) < 0.7 * p_.emergency_stop && cmd.linear.x > 0.0) {
    cmd.linear.x = 0.0;
  }
  cmd_pub_->publish(cmd);
  std_msgs::msg::String st;
  st.data = name(state_);
  state_pub_->publish(st);
}

void WallFollowerController::stop()
{
  RCLCPP_INFO(get_logger(), "Shutting down - stopping robot");
  cmd_pub_->publish(Twist());
}

}  // namespace wall_following
