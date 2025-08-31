#ifndef WALL_DETECTOR_HPP
#define WALL_DETECTOR_HPP

#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <vector>

/**
 * @brief Wall Detection algorithms for autonomous wall following
 * 
 * This class provides methods to detect walls using laser scan data,
 * determine wall consistency, and find the best wall to follow.
 */
class WallDetector {
public:
    /**
     * @brief Constructor for Wall Detector
     */
    WallDetector();
    
    /**
     * @brief Get range measurement at specific angle
     * @param laser_data Laser scan message
     * @param angle Angle in degrees (-180 to +180)
     * @param tolerance Angle tolerance in degrees
     * @return Minimum valid range in specified angle range
     */
    double getRangeAtAngle(const sensor_msgs::msg::LaserScan::SharedPtr laser_data, 
                          double angle, double tolerance = 10.0);
    
    /**
     * @brief Detect wall on specified side of robot
     * @param laser_data Laser scan message
     * @param side Side to check ("right" or "left")
     * @return Pair of (wall_found, average_distance)
     */
    std::pair<bool, double> detectWallOnSide(const sensor_msgs::msg::LaserScan::SharedPtr laser_data, 
                                            const std::string& side);
    
    /**
     * @brief Find the best wall to follow
     * @param laser_data Laser scan message
     * @return Pair of (wall_side, distance) - side is "right", "left", or "none"
     */
    std::pair<std::string, double> findBestWall(const sensor_msgs::msg::LaserScan::SharedPtr laser_data);
    
    /**
     * @brief Set wall detection parameters
     * @param wall_range Maximum range to consider for wall detection
     * @param consistency Maximum allowed variation in wall distance measurements
     */
    void setParameters(double wall_range, double consistency);

private:
    double wall_range_;      ///< Maximum range for wall detection (m)
    double consistency_;     ///< Maximum allowed variation in wall measurements (m)
    
    /**
     * @brief Check if a range value is valid
     * @param range Range value to check
     * @return True if range is valid (not NaN, not Inf, within reasonable bounds)
     */
    bool isValidRange(double range) const;
    
    /**
     * @brief Get multiple range measurements around an angle
     * @param laser_data Laser scan message
     * @param center_angle Center angle in radians
     * @param tolerance_indices Number of indices to include on each side
     * @return Vector of valid range measurements
     */
    std::vector<double> getRangesAroundAngle(const sensor_msgs::msg::LaserScan::SharedPtr laser_data,
                                           double center_angle, int tolerance_indices);
};

#endif // WALL_DETECTOR_HPP