#include "wall_following_cpp_project/pid_controller.hpp"
#include <algorithm>
#include <cmath>

PIDController::PIDController(double kp, double ki, double kd, double dt)
    : kp_(kp), ki_(ki), kd_(kd), dt_(dt), prev_error_(0.0), integral_(0.0) {
}

double PIDController::update(double error) {
    // Proportional term
    double p_term = kp_ * error;
    
    // Integral term with anti-windup
    integral_ += error * dt_;
    // Clamp integral to prevent windup
    integral_ = std::max(INTEGRAL_MIN, std::min(INTEGRAL_MAX, integral_));
    double i_term = ki_ * integral_;
    
    // Derivative term with filtering
    double derivative = 0.0;
    if (!std::isnan(prev_error_) && !std::isinf(prev_error_)) {
        derivative = (error - prev_error_) / dt_;
        // Filter extreme derivatives
        derivative = std::max(-10.0, std::min(10.0, derivative));
    }
    double d_term = kd_ * derivative;
    
    // Calculate total output
    double output = p_term + i_term + d_term;
    
    // Store error for next iteration
    prev_error_ = error;
    
    // Clamp output to reasonable range
    output = std::max(-1.0, std::min(1.0, output));
    
    return output;
}

void PIDController::reset() {
    prev_error_ = 0.0;
    integral_ = 0.0;
}

void PIDController::setGains(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    reset();  // Reset when gains change
}