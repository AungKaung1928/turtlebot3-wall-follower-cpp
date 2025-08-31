#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

/**
 * @brief PID Controller for wall following robot
 * 
 * This class implements a PID (Proportional-Integral-Derivative) controller
 * for maintaining desired distance from walls during wall following behavior.
 */
class PIDController {
public:
    /**
     * @brief Constructor for PID Controller
     * @param kp Proportional gain
     * @param ki Integral gain 
     * @param kd Derivative gain
     * @param dt Time step
     */
    PIDController(double kp = 0.5, double ki = 0.0, double kd = 0.1, double dt = 0.1);
    
    /**
     * @brief Update PID controller with current error
     * @param error Current error value
     * @return Control output
     */
    double update(double error);
    
    /**
     * @brief Reset PID controller internal state
     */
    void reset();
    
    /**
     * @brief Set PID gains
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void setGains(double kp, double ki, double kd);

private:
    double kp_;          ///< Proportional gain
    double ki_;          ///< Integral gain
    double kd_;          ///< Derivative gain
    double dt_;          ///< Time step
    double prev_error_;  ///< Previous error for derivative calculation
    double integral_;    ///< Integral term accumulator
    
    // Integral windup limits
    static constexpr double INTEGRAL_MAX = 1.0;
    static constexpr double INTEGRAL_MIN = -1.0;
};

#endif // PID_CONTROLLER_HPP