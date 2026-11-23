#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "wall_following_cpp_project/wall_detector.hpp"

using wall_following::WallDetector;

namespace
{
// 360-ray scan of one straight wall. Its normal points from the robot at bearing
// normal_deg (robot frame, 0 = front, -90 = right) at perpendicular distance d:
// a ray at bearing phi returns d / cos(phi - normal), no return when it points away.
WallDetector::Scan wallScan(double angle_min, double normal_deg, double d)
{
  WallDetector::Scan scan;
  scan.angle_min = angle_min;
  scan.angle_increment = 2.0 * M_PI / 360.0;
  scan.range_min = 0.12;
  scan.range_max = 3.5;
  const double normal = normal_deg * M_PI / 180.0;
  for (int i = 0; i < 360; ++i) {
    const double c = std::cos(angle_min + i * scan.angle_increment - normal);
    const double r = c > 0.0 ? d / c : std::numeric_limits<double>::infinity();
    scan.ranges.push_back(static_cast<float>(r < scan.range_max ? r : INFINITY));
  }
  return scan;
}
}  // namespace

// 0 = TurtleBot3 LDS-01 (0..2pi), -pi = the other common layout.
class WallDetectorLayouts : public ::testing::TestWithParam<double> {};

TEST_P(WallDetectorLayouts, ParallelWallOnTheRight)
{
  const WallDetector det(40.0);  // config/wall_following_params.yaml beam_spread_deg
  const auto scan = wallScan(GetParam(), -90.0, 0.5);
  EXPECT_NEAR(det.rangeAt(scan, -90.0, false), 0.5, 1e-3);
  EXPECT_FALSE(std::isfinite(det.rangeAt(scan, 90.0, false)));  // nothing on the left
  const auto est = det.estimate(scan, -1);
  ASSERT_TRUE(est.has_value());
  EXPECT_NEAR(est->distance, 0.5, 5e-3);
  EXPECT_NEAR(est->angle, 0.0, 1e-2);
  EXPECT_FALSE(det.estimate(scan, +1).has_value());  // left wall missing
}

TEST_P(WallDetectorLayouts, ConvergingWallGivesPositiveAngle)
{
  // Normal swung 0.2 rad toward the front: the heading closes on the wall.
  const double alpha = 0.2;
  const WallDetector det(40.0);
  const auto scan = wallScan(GetParam(), -90.0 + alpha * 180.0 / M_PI, 0.5);
  const auto est = det.estimate(scan, -1);
  ASSERT_TRUE(est.has_value());
  EXPECT_NEAR(est->angle, alpha, 1e-2);
  EXPECT_NEAR(est->distance, 0.5, 5e-3);
}

TEST_P(WallDetectorLayouts, MinInArcSeesTheFrontWall)
{
  const WallDetector det(40.0);
  const auto scan = wallScan(GetParam(), 0.0, 0.3);
  EXPECT_NEAR(det.minInArc(scan, -30, 30, 3), 0.3, 1e-3);
  EXPECT_FALSE(std::isfinite(det.minInArc(scan, 150, 210, 3)));  // behind: open
}

INSTANTIATE_TEST_SUITE_P(ScanLayouts, WallDetectorLayouts, ::testing::Values(0.0, -M_PI));
