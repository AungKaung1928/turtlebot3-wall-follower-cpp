#pragma once

#include <memory>
#include <string>
#include <utility>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>

#include "wall_following_cpp_project/pid_controller.hpp"
#include "wall_following_cpp_project/wall_detector.hpp"

namespace wall_following
{

enum class State { SEARCHING, FOLLOWING, AVOIDING };

/// Reactive wall follower: SEARCHING -> FOLLOWING (PD on look-ahead wall distance) with
/// AVOIDING pre-empting both when the front or side zones are blocked.
class WallFollowerController : public rclcpp::Node
{
public:
  explicit WallFollowerController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// Publish a zero Twist. Called once from main() after the executor stops.
  void stop();

private:
  struct Params
  {
    double desired_distance;
    double forward_speed;
    double search_speed;
    double max_angular_speed;
    double kp;
    double kd;
    double lookahead;
    double beam_spread_deg;
    double emergency_stop;
    double slow_down;
    double wall_min;
    double wall_lost;
    double side_clearance;
    int search_period;
    int stuck_threshold;
    double control_frequency;
  };

  void declareParameters();
  double paramInRange(const std::string & name, double lo, double hi);
  void loadParameters();

  void laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  void controlLoop();

  std::pair<bool, double> detectCollisionThreat(double margin) const;
  geometry_msgs::msg::Twist searchForWall();
  geometry_msgs::msg::Twist followWall();
  geometry_msgs::msg::Twist avoidCollision();
  int pickEscapeDirection() const;
  void setState(State next, const std::string & reason);
  static const char * name(State s);

  Params p_{};
  std::unique_ptr<WallDetector> detector_;
  std::unique_ptr<PidController> pid_;

  State state_{State::SEARCHING};
  State resume_state_{State::SEARCHING};
  int wall_sign_{-1};  ///< -1 = wall on the right, +1 = wall on the left
  int search_direction_{1};
  int escape_dir_{1};
  int state_counter_{0};
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace wall_following
