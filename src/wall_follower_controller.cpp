#include "wall_following_cpp_project/wall_follower_controller.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

using namespace std::chrono_literals;

WallFollowerController::WallFollowerController() 
    : Node("wall_follower_controller") {
    
    // Parameters - ADJUSTED FOR SAFETY
    desired_distance_ = 0.6;      // Increased from 0.5 for more clearance
    forward_speed_ = 0.20;        // Slightly reduced for better control
    search_speed_ = 0.16;
    max_angular_speed_ = 0.6;
    kp_ = 1.8; kd_ = 0.7;        // More responsive control
    
    // Safety distances - INCREASED FOR NO CONTACT
    emergency_stop_ = 0.55;       // Increased from 0.35 - much earlier detection
    slow_down_dist_ = 0.8;        // Increased from 0.5
    wall_min_ = 0.45;            // Increased from 0.35 - minimum wall distance
    wall_lost_ = 1.5;            // Increased from 1.2
    side_clearance_ = 0.4;       // Increased from 0.25 - side obstacle clearance
    
    // State initialization
    following_wall_ = false;
    wall_side_ = "right";
    laser_data_ = nullptr;
    prev_error_ = 0.0;
    search_dir_ = 1;
    counter_ = 0;
    stuck_counter_ = 0;
    
    // Initialize helper objects
    wall_detector_ = std::make_unique<WallDetector>();
    pid_controller_ = std::make_unique<PIDController>(kp_, 0.0, kd_, 0.05);
    
    // ROS2 publishers and subscribers
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, 
        std::bind(&WallFollowerController::laserCallback, this, std::placeholders::_1));
    
    // Control timer (20 Hz)
    timer_ = this->create_wall_timer(50ms, std::bind(&WallFollowerController::controlLoop, this));
    
    RCLCPP_INFO(this->get_logger(), "Wall Follower - Enhanced Safety Mode");
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
    
    // Convert angle to radians
    double angle_rad = angle * M_PI / 180.0;
    
    // Calculate index
    int idx = static_cast<int>((angle_rad - laser_data_->angle_min) / laser_data_->angle_increment);
    
    // Get multiple readings for averaging (wider range)
    int start_idx = std::max(0, idx - 3);
    int end_idx = std::min(static_cast<int>(laser_data_->ranges.size()) - 1, idx + 3);
    
    std::vector<double> dists;
    for (int i = start_idx; i <= end_idx; ++i) {
        double d = laser_data_->ranges[i];
        if (!isInvalidFloat(d) && d > 0.05 && d < 3.5) {
            dists.push_back(d);
        }
    }
    
    if (dists.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    
    if (avg) {
        // Return average
        double sum = std::accumulate(dists.begin(), dists.end(), 0.0);
        return sum / dists.size();
    } else {
        // Return minimum
        return *std::min_element(dists.begin(), dists.end());
    }
}

double WallFollowerController::getMinInRange(int start, int end, int step) {
    std::vector<double> dists;
    for (int angle = start; angle <= end; angle += step) {
        double d = getDist(static_cast<double>(angle));
        if (!isInvalidFloat(d)) {
            dists.push_back(d);
        }
    }
    
    if (dists.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    
    return *std::min_element(dists.begin(), dists.end());
}

std::pair<bool, double> WallFollowerController::checkCollision() {
    // Check front zones more thoroughly
    double front_center = getMinInRange(-20, 20, 2);
    double front_left = getMinInRange(20, 50, 3);
    double front_right = getMinInRange(-50, -20, 3);
    double front_wide_left = getMinInRange(50, 70, 5);
    double front_wide_right = getMinInRange(-70, -50, 5);
    
    double front = std::min({front_center, front_left, front_right});
    
    // Check sides with increased clearance
    double right = getMinInRange(-90, -60, 3);
    double left = getMinInRange(60, 90, 3);
    
    // Collision if any zone is too close
    bool collision = (front < emergency_stop_ ||
                     right < side_clearance_ ||
                     left < side_clearance_ ||
                     front_wide_left < 0.35 ||
                     front_wide_right < 0.35);
    
    return std::make_pair(collision, front);
}

bool WallFollowerController::findWall() {
    double right = getDist(-90.0, true);
    double left = getDist(90.0, true);
    
    if (right < wall_lost_ || left < wall_lost_) {
        wall_side_ = (right < left) ? "right" : "left";
        following_wall_ = true;
        counter_ = 0;
        stuck_counter_ = 0;
        RCLCPP_INFO(this->get_logger(), "Following %s wall", wall_side_.c_str());
        return true;
    }
    
    return false;
}

geometry_msgs::msg::Twist WallFollowerController::wallFollow() {
    geometry_msgs::msg::Twist cmd;
    
    // Get wall distance
    double angle = (wall_side_ == "right") ? -90.0 : 90.0;
    double wall_dist = getDist(angle, true);
    
    // Lost wall?
    if (wall_dist > wall_lost_) {
        following_wall_ = false;
        return search();
    }
    
    // Too close to wall? More aggressive avoidance
    if (wall_dist < wall_min_) {
        cmd.linear.x = 0.08;  // Slower when too close
        cmd.angular.z = (wall_side_ == "right") ? 0.6 : -0.6;  // More aggressive turn
        return cmd;
    }
    
    // PID control with enhanced responsiveness
    double error = desired_distance_ - wall_dist;
    double angular = kp_ * error + kd_ * (error - prev_error_) / 0.05;
    
    if (wall_side_ == "left") {
        angular = -angular;
    }
    
    cmd.angular.z = clamp(angular, -max_angular_speed_, max_angular_speed_);
    
    // Speed control based on front clearance with more conservative scaling
    double front = getMinInRange(-30, 30, 2);
    if (front < slow_down_dist_) {
        // More conservative speed reduction
        double factor = std::max(0.2, (front - emergency_stop_) / (slow_down_dist_ - emergency_stop_));
        cmd.linear.x = forward_speed_ * factor;
    } else {
        cmd.linear.x = forward_speed_;
    }
    
    // Further slow down for sharp turns
    if (std::abs(cmd.angular.z) > 0.3) {
        cmd.linear.x *= (1.0 - std::abs(cmd.angular.z) / max_angular_speed_ * 0.7);  // More aggressive slowdown
    }
    
    cmd.linear.x = std::max(cmd.linear.x, 0.05);
    prev_error_ = error;
    
    return cmd;
}

geometry_msgs::msg::Twist WallFollowerController::search() {
    geometry_msgs::msg::Twist cmd;
    double front = getMinInRange(-25, 25, 2);
    
    // Speed based on front clearance
    if (front > slow_down_dist_) {
        cmd.linear.x = search_speed_;
    } else {
        cmd.linear.x = search_speed_ * 0.4;
    }
    
    // Search pattern with stuck detection
    counter_++;
    if (counter_ > 60) {  // Longer search periods
        search_dir_ *= -1;
        counter_ = 0;
        stuck_counter_++;
    }
    
    // If stuck for too long, try more aggressive maneuver
    if (stuck_counter_ > 3) {
        cmd.angular.z = 0.6 * search_dir_;
        cmd.linear.x = 0.1;
        if (counter_ > 30) {  // Reset stuck counter after trying
            stuck_counter_ = 0;
        }
    } else {
        cmd.angular.z = 0.25 * search_dir_;
    }
    
    return cmd;
}

geometry_msgs::msg::Twist WallFollowerController::escapeCollision() {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;  // Stop immediately
    
    // Analyze escape directions
    double left_clearance = getMinInRange(45, 135, 5);
    double right_clearance = getMinInRange(-135, -45, 5);
    double back_left = getMinInRange(135, 180, 5);
    double back_right = getMinInRange(-180, -135, 5);
    
    // Choose best escape direction
    if (left_clearance > right_clearance && left_clearance > 0.8) {
        cmd.angular.z = 0.8;  // Turn left aggressively
    } else if (right_clearance > 0.8) {
        cmd.angular.z = -0.8;  // Turn right aggressively
    } else if (back_left > back_right && back_left > 0.6) {
        cmd.angular.z = 0.8;  // Turn toward back-left
    } else if (back_right > 0.6) {
        cmd.angular.z = -0.8;  // Turn toward back-right
    } else {
        // If no good escape, rotate to find opening
        cmd.angular.z = (counter_ % 40 < 20) ? 0.6 : -0.6;
    }
    
    return cmd;
}

void WallFollowerController::controlLoop() {
    if (!laser_data_) {
        return;
    }
    
    // Enhanced collision check
    auto [collision, min_dist] = checkCollision();
    if (collision) {
        geometry_msgs::msg::Twist cmd = escapeCollision();
        cmd_pub_->publish(cmd);
        if (counter_ % 10 == 0) {  // Reduce log frequency
            RCLCPP_WARN(this->get_logger(), "Collision avoidance! %.2fm - Escaping", min_dist);
        }
        counter_++;
        return;
    }
    
    // Reset counter when not in collision
    counter_ = 0;
    
    // Main navigation logic
    geometry_msgs::msg::Twist cmd;
    if (following_wall_) {
        cmd = wallFollow();
    } else if (findWall()) {
        cmd = wallFollow();
    } else {
        cmd = search();
    }
    
    // Enhanced safety limits
    cmd.linear.x = clamp(cmd.linear.x, 0.0, forward_speed_);
    cmd.angular.z = clamp(cmd.angular.z, -max_angular_speed_, max_angular_speed_);
    
    // Final safety check before publishing
    double immediate_front = getMinInRange(-15, 15, 1);
    if (immediate_front < 0.4) {  // Emergency brake
        cmd.linear.x = 0.0;
    }
    
    cmd_pub_->publish(cmd);
}