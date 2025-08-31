#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <csignal>

#include "wall_following_cpp_project/wall_follower_controller.hpp"

// Global pointer to the node for signal handling
std::shared_ptr<WallFollowerController> g_node = nullptr;

/**
 * @brief Signal handler for clean shutdown
 * @param sig Signal number
 */
void signalHandler(int sig) {
    (void)sig;  // Suppress unused parameter warning
    
    if (g_node) {
        // Stop the robot by publishing zero velocity
        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.linear.z = 0.0;
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = 0.0;
        
        RCLCPP_INFO(g_node->get_logger(), "Shutting down wall follower - stopping robot");
        
        // Note: We would publish the stop command here, but we need access to the publisher
        // The actual stop command will be handled in the node destructor or cleanup
    }
    
    rclcpp::shutdown();
}

/**
 * @brief Main function - entry point for the wall follower controller
 * @param argc Argument count
 * @param argv Argument values
 * @return Exit status
 */
int main(int argc, char* argv[]) {
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    try {
        // Create the wall follower controller node
        g_node = std::make_shared<WallFollowerController>();
        
        // Set up signal handlers for clean shutdown
        std::signal(SIGINT, signalHandler);
        std::signal(SIGTERM, signalHandler);
        
        RCLCPP_INFO(g_node->get_logger(), "Starting Wall Follower Controller");
        RCLCPP_INFO(g_node->get_logger(), "Press Ctrl+C to stop the robot safely");
        
        // Spin the node
        rclcpp::spin(g_node);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("wall_follower_main"), 
                     "Exception in wall follower: %s", e.what());
        return 1;
    }
    
    // Clean shutdown
    if (g_node) {
        RCLCPP_INFO(g_node->get_logger(), "Wall follower controller shutting down");
        g_node.reset();
    }
    
    rclcpp::shutdown();
    return 0;
}