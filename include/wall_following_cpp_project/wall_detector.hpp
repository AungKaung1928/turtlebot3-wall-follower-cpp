#pragma once

#include <cstddef>
#include <optional>

#include <sensor_msgs/msg/laser_scan.hpp>

namespace wall_following
{

/// Perpendicular distance to the followed wall and the heading angle to it.
/// angle > 0 means the current heading converges on the wall.
struct WallEstimate
{
  double distance;
  double angle;
};

/// Pure scan geometry. Bearings are in the robot frame: 0 = front, -90 = right, +90 = left.
/// Works for both LaserScan conventions (angle_min = 0 and angle_min = -pi) by wrapping the
/// bearing offset modulo a full turn.
class WallDetector
{
public:
  using Scan = sensor_msgs::msg::LaserScan;

  explicit WallDetector(double beam_spread_deg);

  /// Range at one bearing; average = mean of the +-3 neighbouring valid returns. inf if none.
  double rangeAt(const Scan & scan, double angle_deg, bool average) const;

  /// Minimum valid range over [start, end] degrees sampled every step degrees.
  double minInArc(const Scan & scan, int start_deg, int end_deg, int step_deg) const;

  /// Two-beam wall estimate on one side (side_sign -1 = right, +1 = left).
  /// nullopt when either beam has no return, i.e. the wall ended beside the robot.
  std::optional<WallEstimate> estimate(const Scan & scan, int side_sign) const;

private:
  double beam_spread_deg_;

  static std::size_t indexFor(const Scan & scan, double angle_deg);
  static double validRange(const Scan & scan, std::size_t idx);
};

}  // namespace wall_following
