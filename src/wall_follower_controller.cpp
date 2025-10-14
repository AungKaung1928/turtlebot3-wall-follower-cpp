#include "wall_following_cpp_project/wall_follower_controller.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

using namespace std::chrono_literals;

WallFollowerController::WallFollowerController() 
    : Node("wall_follower_controller") {
    
    // TUNED PARAMETERS FOR TURTLEBOT3 WORLD
    desired_distance_ = 0.8;      // Target wall distance
    forward_speed_ = 0.15;        // Forward speed
    search_speed_ = 0.1;          // Search speed
    max_angular_speed_ = 0.6;     // Max rotation
    kp_ = 3.0; kd_ = 0.5;         // PID gains
    
    // ADJUSTED SAFETY DISTANCES
    emergency_stop_ = 0.25;       // Only stop if VERY close
    slow_down_dist_ = 0.5;        // Slow down distance
    wall_min_ = 0.45;              // Minimum wall distance
    wall_lost_ = 1.0;             // Wall lost threshold
    side_clearance_ = 0.3;        // Side clearance
    
    // State
    following_wall_ = false;
    wall_side_ = "right";
    laser_data_ = nullptr;
    prev_error_ = 0.0;
    search_dir_ = 1;
    counter_ = 0;
    stuck_counter_ = 0;
    
    // Initialize helpers
    wall_detector_ = std::make_unique<WallDetector>();
    pid_controller_ = std::make_unique<PIDController>(kp_, 0.0, kd_, 0.05);
    
    // ROS2
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, 
        std::bind(&WallFollowerController::laserCallback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(50ms, std::bind(&WallFollowerController::controlLoop, this));
    
    RCLCPP_INFO(this->get_logger(), "Wall Follower Started - Simple Mode");
}

void WallFollowerController::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    laser_data_ = msg;
}

bool WallFollowerController::isInvalidFloat(double value) {
    return std::isnan(value) || std::isinf(value);
}

double WallFollowerController::clamp(double value, double min_val, double max_val) {
    return std::max(min_val, std::min(value, max_val));
}

double WallFollowerController::getDist(double angle, bool avg) {
    if (!laser_data_ || laser_data_->ranges.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    
    // Convert angle to radians and normalize
    double angle_rad = angle * M_PI / 180.0;
    while (angle_rad > M_PI) angle_rad -= 2 * M_PI;
    while (angle_rad < -M_PI) angle_rad += 2 * M_PI;
    
    // Calculate index
    int idx = static_cast<int>((angle_rad - laser_data_->angle_min) / laser_data_->angle_increment);
    
    // Wrap around for angles
    int total_readings = laser_data_->ranges.size();
    if (idx < 0) idx += total_readings;
    if (idx >= total_readings) idx -= total_readings;
    
    // Get reading(s)
    if (!avg) {
        // Single reading
        if (idx >= 0 && idx < total_readings) {
            double range = laser_data_->ranges[idx];
            if (range > 0.1 && range < 3.5 && !isInvalidFloat(range)) {
                return range;
            }
        }
        return std::numeric_limits<double>::infinity();
    } else {
        // Average multiple readings for stability
        std::vector<double> valid_ranges;
        for (int offset = -3; offset <= 3; offset++) {
            int check_idx = idx + offset;
            if (check_idx < 0) check_idx += total_readings;
            if (check_idx >= total_readings) check_idx -= total_readings;
            
            if (check_idx >= 0 && check_idx < total_readings) {
                double range = laser_data_->ranges[check_idx];
                if (range > 0.1 && range < 3.5 && !isInvalidFloat(range)) {
                    valid_ranges.push_back(range);
                }
            }
        }
        
        if (valid_ranges.empty()) {
            return std::numeric_limits<double>::infinity();
        }
        
        // Return average
        double sum = 0.0;
        for (double r : valid_ranges) sum += r;
        return sum / valid_ranges.size();
    }
}

double WallFollowerController::getMinInRange(int start, int end, int step) {
    double min_dist = std::numeric_limits<double>::infinity();
    for (int angle = start; angle <= end; angle += step) {
        double d = getDist(static_cast<double>(angle), false);
        if (d < min_dist) min_dist = d;
    }
    return min_dist;
}

std::pair<bool, double> WallFollowerController::checkCollision() {
    // Only check narrow front for real obstacles
    double front = getMinInRange(-10, 10, 2);
    return std::make_pair(front < emergency_stop_, front);
}

bool WallFollowerController::findWall() {
    if (!laser_data_) return false;
    
    // Simple approach - check for walls on sides
    double right = getDist(-90.0, true);
    double left = getDist(90.0, true);
    double front = getDist(0.0, true);
    
    // If wall in front, need to turn
    if (front < 0.5) {
        return false;  // Don't start following yet
    }
    
    // Prefer right wall
    if (right < 0.8) {
        wall_side_ = "right";
        following_wall_ = true;
        RCLCPP_INFO(this->get_logger(), "Following RIGHT wall at %.2fm", right);
        return true;
    } else if (left < 0.8) {
        wall_side_ = "left";
        following_wall_ = true;
        RCLCPP_INFO(this->get_logger(), "Following LEFT wall at %.2fm", left);
        return true;
    }
    
    return false;
}

geometry_msgs::msg::Twist WallFollowerController::wallFollow() {
    geometry_msgs::msg::Twist cmd;
    
    // KEY DISTANCES for wall following algorithm
    double side_angle = (wall_side_ == "right") ? -90.0 : 90.0;
    double diag_angle = (wall_side_ == "right") ? -45.0 : 45.0;
    
    double side_dist = getDist(side_angle, true);
    double diag_dist = getDist(diag_angle, true);
    double front_dist = getDist(0.0, true);
    
    // Lost wall check
    if (side_dist > wall_lost_ || isInvalidFloat(side_dist)) {
        following_wall_ = false;
        RCLCPP_INFO(this->get_logger(), "Wall lost");
        return search();
    }
    
    // SIMPLE WALL FOLLOWING LOGIC
    
    // 1. If wall/obstacle in front, turn away
    if (front_dist < 0.55) {
        cmd.linear.x = 0.0;
        cmd.angular.z = (wall_side_ == "right") ? 0.5 : -0.5;
        return cmd;
    }
    
    // 2. If too close to wall, turn away
    if (side_dist < wall_min_) {
        cmd.linear.x = 0.05;
        cmd.angular.z = (wall_side_ == "right") ? 0.3 : -0.3;
        return cmd;
    }
    
    // 3. Normal wall following - simple proportional control
    double error = side_dist - desired_distance_;
    
    // Simple P control (positive error = too far, negative = too close)
    double angular = -2.0 * error;  // Negative because we want to turn toward wall when too far
    
    if (wall_side_ == "left") {
        angular = -angular;  // Reverse for left wall
    }
    
    // 4. Use diagonal sensor to anticipate corners
    if (diag_dist < side_dist * 0.7 && diag_dist < 0.6) {
        // Corner ahead - start turning
        if (wall_side_ == "right") {
            angular = std::max(angular, 0.3);  // Turn left for right wall
        } else {
            angular = std::min(angular, -0.3);  // Turn right for left wall
        }
    }
    
    // Set commands
    cmd.angular.z = clamp(angular, -max_angular_speed_, max_angular_speed_);
    
    // Adjust speed based on turning
    if (std::abs(cmd.angular.z) > 0.3) {
        cmd.linear.x = forward_speed_ * 0.5;  // Slow down in turns
    } else {
        cmd.linear.x = forward_speed_;
    }
    
    return cmd;
}

geometry_msgs::msg::Twist WallFollowerController::search() {
    geometry_msgs::msg::Twist cmd;
    
    double front = getDist(0.0, true);
    
    // Simple search - move forward and rotate
    if (front > 0.5) {
        cmd.linear.x = search_speed_;
        cmd.angular.z = 0.3;  // Gentle rotation while moving
    } else {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.5;  // Just rotate if blocked
    }
    
    return cmd;
}

geometry_msgs::msg::Twist WallFollowerController::escapeCollision() {
    geometry_msgs::msg::Twist cmd;
    
    // Simple escape - back up and turn
    cmd.linear.x = -0.05;  // Back up slowly
    
    // Turn away from obstacle
    double left = getDist(90.0, false);
    double right = getDist(-90.0, false);
    
    if (left > right) {
        cmd.angular.z = 0.5;  // Turn left
    } else {
        cmd.angular.z = -0.5;  // Turn right
    }
    
    return cmd;
}

void WallFollowerController::controlLoop() {
    if (!laser_data_) {
        return;
    }
    
    geometry_msgs::msg::Twist cmd;
    
    // Simple state machine
    auto [collision, front_dist] = checkCollision();
    
    if (collision && front_dist < 0.2) {
        // Only escape if REALLY close
        cmd = escapeCollision();
    } else if (following_wall_) {
        cmd = wallFollow();
    } else if (findWall()) {
        cmd = wallFollow();
    } else {
        cmd = search();
    }
    
    // Safety limits
    cmd.linear.x = clamp(cmd.linear.x, -0.1, forward_speed_);
    cmd.angular.z = clamp(cmd.angular.z, -max_angular_speed_, max_angular_speed_);
    
    // Publish
    cmd_pub_->publish(cmd);
}