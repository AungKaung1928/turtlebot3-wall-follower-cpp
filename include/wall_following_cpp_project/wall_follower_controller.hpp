#ifndef WALL_FOLLOWER_CONTROLLER_HPP
#define WALL_FOLLOWER_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <string>

#include "wall_following_cpp_project/wall_detector.hpp"
#include "wall_following_cpp_project/pid_controller.hpp"

/**
 * @brief Main Wall Following Controller Node
 * 
 * This class implements the main control logic for autonomous wall following
 * behavior with enhanced safety features and collision avoidance.
 */
class WallFollowerController : public rclcpp::Node {
public:
    /**
     * @brief Constructor for Wall Follower Controller
     */
    WallFollowerController();

private:
    // Safety and control parameters
    double desired_distance_;     ///< Target distance from wall (m)
    double forward_speed_;        ///< Normal forward speed (m/s)
    double search_speed_;         ///< Speed during wall search (m/s)
    double max_angular_speed_;    ///< Maximum angular velocity (rad/s)
    double kp_, kd_;             ///< PID gains for wall following
    
    // Safety distances - INCREASED FOR NO CONTACT
    double emergency_stop_;       ///< Emergency brake distance (m)
    double slow_down_dist_;       ///< Distance to start slowing down (m)
    double wall_min_;            ///< Minimum allowed distance to wall (m)
    double wall_lost_;           ///< Distance threshold for losing wall (m)
    double side_clearance_;      ///< Minimum side obstacle clearance (m)
    
    // State variables
    bool following_wall_;        ///< Currently following a wall
    std::string wall_side_;      ///< Side of wall being followed ("right" or "left")
    sensor_msgs::msg::LaserScan::SharedPtr laser_data_;  ///< Latest laser scan data
    double prev_error_;          ///< Previous error for derivative control
    int search_dir_;             ///< Search direction (+1 or -1)
    int counter_;                ///< General purpose counter
    int stuck_counter_;          ///< Counter for stuck detection
    
    // ROS2 components
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Helper objects
    std::unique_ptr<WallDetector> wall_detector_;
    std::unique_ptr<PIDController> pid_controller_;
    
    // Callback functions
    /**
     * @brief Laser scan callback
     * @param msg Laser scan message
     */
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    /**
     * @brief Main control loop callback
     */
    void controlLoop();
    
    // Control methods
    /**
     * @brief Get distance measurement at specific angle
     * @param angle Angle in degrees
     * @param avg Whether to average multiple readings
     * @return Distance measurement
     */
    double getDist(double angle, bool avg = false);
    
    /**
     * @brief Get minimum distance in angle range
     * @param start Start angle (degrees)
     * @param end End angle (degrees)
     * @param step Step size (degrees)
     * @return Minimum distance in range
     */
    double getMinInRange(int start, int end, int step = 2);
    
    /**
     * @brief Enhanced collision detection
     * @return Pair of (collision_detected, front_distance)
     */
    std::pair<bool, double> checkCollision();
    
    /**
     * @brief Find nearest wall to follow
     * @return True if wall found and following started
     */
    bool findWall();
    
    /**
     * @brief Execute wall following behavior
     * @return Twist command for wall following
     */
    geometry_msgs::msg::Twist wallFollow();
    
    /**
     * @brief Execute wall search behavior
     * @return Twist command for searching
     */
    geometry_msgs::msg::Twist search();
    
    /**
     * @brief Execute collision escape behavior
     * @return Twist command for escaping collision
     */
    geometry_msgs::msg::Twist escapeCollision();
    
    // Utility methods
    /**
     * @brief Check if a value is NaN or Inf
     * @param value Value to check
     * @return True if value is NaN or Inf
     */
    bool isInvalidFloat(double value);
    
    /**
     * @brief Clamp value between min and max
     * @param value Value to clamp
     * @param min_val Minimum value
     * @param max_val Maximum value
     * @return Clamped value
     */
    double clamp(double value, double min_val, double max_val);
};

#endif // WALL_FOLLOWER_CONTROLLER_HPP