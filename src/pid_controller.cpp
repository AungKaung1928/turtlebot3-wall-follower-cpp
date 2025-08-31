#include "wall_following_cpp_project/pid_controller.hpp"
#include <algorithm>

PIDController::PIDController(double kp, double ki, double kd, double dt)
    : kp_(kp), ki_(ki), kd_(kd), dt_(dt), prev_error_(0.0), integral_(0.0) {
}

double PIDController::update(double error) {
    // Update integral term with windup protection
    integral_ += error * dt_;
    integral_ = std::max(std::min(integral_, INTEGRAL_MAX), INTEGRAL_MIN);
    
    // Calculate derivative term
    double derivative = (error - prev_error_) / dt_;
    
    // Calculate PID output
    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    
    // Store error for next iteration
    prev_error_ = error;
    
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
}