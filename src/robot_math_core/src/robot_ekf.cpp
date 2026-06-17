#include "robot_math_core/robot_ekf.hpp"
#include <cmath>
#include <iostream>

namespace robot_math_core
{

EkfFilter::EkfFilter() : theta_(0.0), P_(0.01) {
    Q_ = 0.005;
    R_ = 0.0001;
}

double EkfFilter::predict(double omega_enc, double dt) {
    theta_ += omega_enc * dt; 
    
    P_ += Q_ * dt; 

    theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

    return theta_;
}

double EkfFilter::update(double omega_gyro, double omega_enc, double dt) {

    double delta_theta_gyro = omega_gyro * dt;
    double delta_theta_enc  = omega_enc * dt;

    double y = delta_theta_gyro - delta_theta_enc;   

    double S = P_ + R_;
    double K = P_ / S;

    theta_ += K * y;

    P_ = (1.0 - K) * P_;

    theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

    std::cout << "Corrected Heading (After EKF): " << theta_ << std::endl;
    return theta_;
}

void EkfFilter::setNoiseParameters(double q_theta, double r_gyro)
{
    Q_ = q_theta;
    R_ = r_gyro;
}

void EkfFilter::resetState(double initial_theta, double initial_P)
{
    theta_ = initial_theta;
    P_ = initial_P;
}

}  // namespace robot_math_core