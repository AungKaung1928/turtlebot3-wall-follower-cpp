#include "wall_following_cpp_project/wall_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace wall_following
{

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kTwoPi = 2.0 * M_PI;
}  // namespace

WallDetector::WallDetector(double beam_spread_deg)
: beam_spread_deg_(beam_spread_deg)
{
}

std::size_t WallDetector::indexFor(const Scan & scan, double angle_deg)
{
  const auto n = scan.ranges.size();
  double offset = std::fmod(angle_deg * M_PI / 180.0 - scan.angle_min, kTwoPi);
  if (offset < 0.0) {
    offset += kTwoPi;
  }
  const auto idx = static_cast<std::size_t>(std::lround(offset / scan.angle_increment));
  return idx % n;
}

double WallDetector::validRange(const Scan & scan, std::size_t idx)
{
  const double r = scan.ranges[idx];
  if (!std::isfinite(r) || r <= scan.range_min || r >= scan.range_max) {
    return kInf;
  }
  return r;
}

double WallDetector::rangeAt(const Scan & scan, double angle_deg, bool average) const
{
  if (scan.ranges.empty()) {
    return kInf;
  }
  const auto n = scan.ranges.size();
  const auto idx = indexFor(scan, angle_deg);
  if (!average) {
    return validRange(scan, idx);
  }
  double sum = 0.0;
  int count = 0;
  for (int k = -3; k <= 3; ++k) {
    const double r = validRange(scan, (idx + n + k) % n);
    if (std::isfinite(r)) {
      sum += r;
      ++count;
    }
  }
  return count > 0 ? sum / count : kInf;
}

double WallDetector::minInArc(const Scan & scan, int start_deg, int end_deg, int step_deg) const
{
  double best = kInf;
  for (int a = start_deg; a <= end_deg; a += step_deg) {
    best = std::min(best, rangeAt(scan, a, false));
  }
  return best;
}

std::optional<WallEstimate> WallDetector::estimate(const Scan & scan, int side_sign) const
{
  const double theta = beam_spread_deg_ * M_PI / 180.0;
  const double b = rangeAt(scan, side_sign * 90.0, true);
  const double a = rangeAt(scan, side_sign * (90.0 - beam_spread_deg_), true);
  if (!std::isfinite(a) || !std::isfinite(b)) {
    return std::nullopt;
  }
  // b = d/cos(alpha), a = d/cos(theta - alpha)  =>  tan(alpha) = (b - a cos theta) / (a sin theta)
  double alpha = std::atan2(b - a * std::cos(theta), a * std::sin(theta));
  alpha = std::clamp(alpha, -1.0, 1.0);
  return WallEstimate{b * std::cos(alpha), alpha};
}

}  // namespace wall_following
