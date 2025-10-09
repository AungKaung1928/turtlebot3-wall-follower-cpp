#include "wall_following_cpp_project/wall_follower_controller.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

using namespace std::chrono_literals;

WallFollowerController::WallFollowerController() 
    : Node("wall_follower_controller") {
    
    // Parameters - OPTIMIZED FOR SAFE WALL FOLLOWING
    desired_distance_ = 0.8;     // Much farther from wall - safe distance
    forward_speed_ = 0.22;       // Good forward speed
    search_speed_ = 0.18;
    max_angular_speed_ = 0.8;    // Can turn faster
    kp_ = 2.0; kd_ = 0.5;        // More responsive PID
    
    // Safety distances - LARGER FOR OBSTACLE AVOIDANCE
    emergency_stop_ = 0.55;      // Stop distance from obstacles
    slow_down_dist_ = 0.85;      // Start slowing down distance
    wall_min_ = 0.50;            // Minimum distance to wall
    wall_lost_ = 2.5;            // Max distance before losing wall
    side_clearance_ = 0.45;      // Side clearance from obstacles
    
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
    
    RCLCPP_INFO(this->get_logger(), "Wall Follower Started - Safe Mode");
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
    
    double angle_rad = angle * M_PI / 180.0;
    int idx = static_cast<int>((angle_rad - laser_data_->angle_min) / laser_data_->angle_increment);
    
    int start_idx = std::max(0, idx - 5);
    int end_idx = std::min(static_cast<int>(laser_data_->ranges.size()) - 1, idx + 5);
    
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
        double sum = std::accumulate(dists.begin(), dists.end(), 0.0);
        return sum / dists.size();
    } else {
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
    // Wide front scanning for obstacles
    double front_center = getMinInRange(-25, 25, 2);
    double front_left = getMinInRange(25, 60, 3);
    double front_right = getMinInRange(-60, -25, 3);
    
    double front = std::min({front_center, front_left, front_right});
    
    bool collision = (front < emergency_stop_);
    
    return std::make_pair(collision, front);
}

bool WallFollowerController::findWall() {
    double right = getDist(-90.0, true);
    double left = getDist(90.0, true);
    double front = getDist(0.0, true);
    
    // Prefer the closer wall if both available
    if (right < wall_lost_ || left < wall_lost_) {
        if (right < left && right < 1.8) {
            wall_side_ = "right";
        } else if (left < 1.8) {
            wall_side_ = "left";
        } else {
            wall_side_ = (right < left) ? "right" : "left";
        }
        
        following_wall_ = true;
        counter_ = 0;
        stuck_counter_ = 0;
        RCLCPP_INFO(this->get_logger(), "Following %s wall at %.2fm", 
                    wall_side_.c_str(), (wall_side_ == "right") ? right : left);
        return true;
    }
    
    return false;
}

geometry_msgs::msg::Twist WallFollowerController::wallFollow() {
    geometry_msgs::msg::Twist cmd;
    
    // Get wall distance
    double angle = (wall_side_ == "right") ? -90.0 : 90.0;
    double wall_dist = getDist(angle, true);
    
    // Also check 45-degree angle for better wall tracking
    double angle_45 = (wall_side_ == "right") ? -45.0 : 45.0;
    double wall_dist_45 = getDist(angle_45, true);
    
    // Get front distance - WIDE CHECK
    double front_0 = getMinInRange(-30, 30, 2);
    double front_left = getMinInRange(30, 70, 3);
    double front_right = getMinInRange(-70, -30, 3);
    double front = std::min({front_0, front_left, front_right});
    
    // Check if wall is lost
    if (wall_dist > wall_lost_) {
        counter_++;
        if (counter_ > 50) {  // Wait 2.5 seconds
            RCLCPP_INFO(this->get_logger(), "Wall lost, searching...");
            following_wall_ = false;
            counter_ = 0;
            return search();
        }
    } else {
        counter_ = 0;
    }
    
    // OBSTACLE AVOIDANCE: If front is blocked, turn AWAY from wall
    if (front < emergency_stop_) {
        cmd.linear.x = 0.0;  // STOP
        cmd.angular.z = (wall_side_ == "right") ? 0.7 : -0.7;  // Turn away from wall
        RCLCPP_WARN(this->get_logger(), "OBSTACLE! Front: %.2fm - Turning away", front);
        return cmd;
    }
    
    // If obstacle detected ahead but not immediate, slow down and prepare to turn
    if (front < slow_down_dist_) {
        cmd.linear.x = 0.12;  // Very slow
        // Turn away from wall preemptively
        cmd.angular.z = (wall_side_ == "right") ? 0.5 : -0.5;
        return cmd;
    }
    
    // Check if getting too close to wall (safety check)
    if (wall_dist < wall_min_) {
        cmd.linear.x = 0.15;
        // Turn away from wall
        cmd.angular.z = (wall_side_ == "right") ? 0.6 : -0.6;
        RCLCPP_INFO(this->get_logger(), "Too close to wall: %.2fm", wall_dist);
        return cmd;
    }
    
    // NORMAL WALL FOLLOWING with PID
    double error = desired_distance_ - wall_dist;
    double angular = kp_ * error + kd_ * (error - prev_error_) / 0.05;
    
    // Reverse for left wall
    if (wall_side_ == "left") {
        angular = -angular;
    }
    
    // Limit angular velocity
    cmd.angular.z = clamp(angular, -max_angular_speed_, max_angular_speed_);
    
    // Forward speed - adjust based on front distance and turning
    if (front < 1.2) {
        double factor = std::max(0.4, (front - emergency_stop_) / (1.2 - emergency_stop_));
        cmd.linear.x = forward_speed_ * factor;
    } else {
        cmd.linear.x = forward_speed_;
    }
    
    // Slow down for sharp turns to maintain control
    if (std::abs(cmd.angular.z) > 0.5) {
        cmd.linear.x *= 0.6;
    }
    
    // Save error for next iteration
    prev_error_ = error;
    
    return cmd;
}

geometry_msgs::msg::Twist WallFollowerController::search() {
    geometry_msgs::msg::Twist cmd;
    
    double front = getMinInRange(-30, 30, 2);
    double left = getDist(90.0, true);
    double right = getDist(-90.0, true);
    
    // ALWAYS MOVE FORWARD while searching (key fix!)
    if (front > 1.0) {
        cmd.linear.x = search_speed_;  // Full speed if clear
    } else if (front > emergency_stop_) {
        cmd.linear.x = search_speed_ * 0.6;  // Slower but keep moving
    } else {
        cmd.linear.x = 0.05;  // Very slow but still moving
    }
    
    counter_++;
    
    // Active exploration strategy
    if (stuck_counter_ > 3) {
        // Been searching too long - force exploration
        // Move forward more aggressively
        cmd.linear.x = 0.20;
        cmd.angular.z = 0.4 * search_dir_;
        
        if (counter_ > 80) {  // 4 seconds
            stuck_counter_ = 0;
            counter_ = 0;
            search_dir_ *= -1;
        }
    } else {
        // Normal search - rotate slowly while moving forward
        if (counter_ > 50) {  // Change direction every 2.5 seconds
            search_dir_ *= -1;
            counter_ = 0;
            stuck_counter_++;
        }
        
        // Check if walls are on sides - turn toward closer wall
        if (right < 2.0 && right < left) {
            cmd.angular.z = -0.3;  // Turn right toward wall
        } else if (left < 2.0 && left < right) {
            cmd.angular.z = 0.3;  // Turn left toward wall
        } else {
            // No wall nearby - explore by turning
            cmd.angular.z = 0.25 * search_dir_;
        }
    }
    
    return cmd;
}

geometry_msgs::msg::Twist WallFollowerController::escapeCollision() {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;  // STOP
    
    // Check all directions
    double left = getMinInRange(60, 120, 5);
    double right = getMinInRange(-120, -60, 5);
    double back = getMinInRange(150, 180, 10);
    
    // Turn toward most open space
    if (left > right && left > 0.7) {
        cmd.angular.z = 0.8;  // Turn left
        RCLCPP_INFO(this->get_logger(), "Escaping - turning left");
    } else if (right > 0.7) {
        cmd.angular.z = -0.8;  // Turn right
        RCLCPP_INFO(this->get_logger(), "Escaping - turning right");
    } else if (back > 0.5) {
        cmd.angular.z = 0.8;  // Turn around
        cmd.linear.x = -0.05;  // Back up slowly
        RCLCPP_INFO(this->get_logger(), "Escaping - backing up");
    } else {
        // Rotate in place to find opening
        cmd.angular.z = (counter_ % 60 < 30) ? 0.8 : -0.8;
    }
    
    return cmd;
}

void WallFollowerController::controlLoop() {
    if (!laser_data_) {
        return;
    }
    
    // Check for immediate collision
    auto [collision, min_dist] = checkCollision();
    
    geometry_msgs::msg::Twist cmd;
    
    // Priority 1: Escape if collision imminent
    if (collision) {
        cmd = escapeCollision();
        counter_++;
    }
    // Priority 2: Follow wall if already following
    else if (following_wall_) {
        cmd = wallFollow();
    }
    // Priority 3: Try to find a wall
    else if (findWall()) {
        cmd = wallFollow();
    }
    // Priority 4: Search for wall
    else {
        cmd = search();
    }
    
    // Final safety limits
    cmd.linear.x = clamp(cmd.linear.x, -0.1, forward_speed_);
    cmd.angular.z = clamp(cmd.angular.z, -max_angular_speed_, max_angular_speed_);
    
    // Publish command
    cmd_pub_->publish(cmd);
}