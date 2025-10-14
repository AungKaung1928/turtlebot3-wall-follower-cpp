#include "wall_following_cpp_project/wall_detector.hpp"
#include <cmath>
#include <limits>
#include <algorithm>
#include <numeric>

WallDetector::WallDetector() 
    : wall_range_(1.5), consistency_(0.3) {
}

bool WallDetector::isValidRange(double range) const {
    return !std::isnan(range) && !std::isinf(range) && range > 0.1 && range < 3.5;
}

std::vector<double> WallDetector::getRangesAroundAngle(
    const sensor_msgs::msg::LaserScan::SharedPtr laser_data,
    double center_angle, int tolerance_indices) {
    
    std::vector<double> valid_ranges;
    
    if (!laser_data || laser_data->ranges.empty()) {
        return valid_ranges;
    }
    
    // Normalize angle to laser scan range
    while (center_angle > M_PI) center_angle -= 2 * M_PI;
    while (center_angle < -M_PI) center_angle += 2 * M_PI;
    
    // Calculate center index
    int center_idx = static_cast<int>((center_angle - laser_data->angle_min) / laser_data->angle_increment);
    
    // Check bounds
    if (center_idx < 0 || center_idx >= static_cast<int>(laser_data->ranges.size())) {
        return valid_ranges;
    }
    
    // Get range of indices to check
    int start_idx = std::max(0, center_idx - tolerance_indices);
    int end_idx = std::min(static_cast<int>(laser_data->ranges.size()) - 1, center_idx + tolerance_indices);
    
    // Collect valid ranges
    for (int i = start_idx; i <= end_idx; ++i) {
        double range = laser_data->ranges[i];
        if (range > laser_data->range_min && range < laser_data->range_max && isValidRange(range)) {
            valid_ranges.push_back(range);
        }
    }
    
    return valid_ranges;
}

double WallDetector::getRangeAtAngle(const sensor_msgs::msg::LaserScan::SharedPtr laser_data, 
                                   double angle, double tolerance) {
    if (!laser_data || laser_data->ranges.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    
    // Convert angle to radians
    double angle_rad = angle * M_PI / 180.0;
    
    // Calculate tolerance in indices
    int tolerance_indices = static_cast<int>((tolerance * M_PI / 180.0) / laser_data->angle_increment);
    tolerance_indices = std::max(1, tolerance_indices);  // At least 1 index
    
    // Get ranges around the target angle
    std::vector<double> valid_ranges = getRangesAroundAngle(laser_data, angle_rad, tolerance_indices);
    
    if (valid_ranges.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    
    // Return average of valid ranges for more stable reading
    double sum = std::accumulate(valid_ranges.begin(), valid_ranges.end(), 0.0);
    return sum / valid_ranges.size();
}

std::pair<bool, double> WallDetector::detectWallOnSide(
    const sensor_msgs::msg::LaserScan::SharedPtr laser_data, 
    const std::string& side) {
    
    // Define angles to check based on side
    std::vector<double> angles;
    if (side == "right") {
        // Check right side angles
        angles = {-90.0, -85.0, -95.0, -45.0};
    } else if (side == "left") {
        // Check left side angles
        angles = {90.0, 85.0, 95.0, 45.0};
    } else {
        return std::make_pair(false, std::numeric_limits<double>::infinity());
    }
    
    // Get distances at each angle
    std::vector<double> distances;
    for (double angle : angles) {
        double dist = getRangeAtAngle(laser_data, angle, 5.0);  // 5 degree tolerance
        if (dist < wall_range_ && dist > 0.2) {  // Minimum distance check
            distances.push_back(dist);
        }
    }
    
    // Need at least 2 valid measurements
    if (distances.size() < 2) {
        return std::make_pair(false, std::numeric_limits<double>::infinity());
    }
    
    // Check consistency
    double min_dist = *std::min_element(distances.begin(), distances.end());
    double max_dist = *std::max_element(distances.begin(), distances.end());
    
    // If measurements are consistent enough, wall is detected
    if (max_dist - min_dist < consistency_ * 2) {
        // Return average distance
        double avg_dist = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
        return std::make_pair(true, avg_dist);
    }
    
    return std::make_pair(false, std::numeric_limits<double>::infinity());
}

std::pair<std::string, double> WallDetector::findBestWall(
    const sensor_msgs::msg::LaserScan::SharedPtr laser_data) {
    
    // Check both sides
    auto [right_found, right_dist] = detectWallOnSide(laser_data, "right");
    auto [left_found, left_dist] = detectWallOnSide(laser_data, "left");
    
    // Prefer the closer wall
    if (right_found && left_found) {
        if (right_dist < left_dist) {
            return std::make_pair("right", right_dist);
        } else {
            return std::make_pair("left", left_dist);
        }
    } else if (right_found) {
        return std::make_pair("right", right_dist);
    } else if (left_found) {
        return std::make_pair("left", left_dist);
    }
    
    return std::make_pair("none", std::numeric_limits<double>::infinity());
}

void WallDetector::setParameters(double wall_range, double consistency) {
    wall_range_ = wall_range;
    consistency_ = consistency;
}