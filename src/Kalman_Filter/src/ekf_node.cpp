#include "robot_estimator/robot_ekf.hpp"
#include <cmath>

RobotEKF::RobotEKF(double dt) : dt_(dt), theta_(0.0) {
    
    // initial the uncertainty or the variance
    P_ = 0.1;

    // Process noise: from the encoder
    Q_ = 0.02;

    // Measurement noise: from the IMU (only gyroscope)
    R_ = 0.005;
}

double RobotEKF::predict(double omega_enc) {
    theta_ += omega_enc * dt_;
    normalizeAngle();

    // we normally have f but as a jacobian d_theta/d_theta which equal to 1
    double F_ = 1;
    P_ = F_*P_*F_ + Q_;

    return theta_;
}

double RobotEKF::update(double omega_gyro, double omega_enc) {
    double y = omega_gyro - omega_enc;

    double S = P_ + R_;

    double K = P_ / S;

    theta_ += K * y * dt_;
    normalizeAngle();

    P_ = (1.0 - K) * P_;

    return theta_;
}

void RobotEKF::normalizeAngle() {
    theta_ = std::atan2(std::sin(theta_), std::cos(theta_));
}