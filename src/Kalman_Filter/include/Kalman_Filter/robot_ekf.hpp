#ifndef ROBOT_ESTIMATOR_ROBOT_EKF_HPP_
#define ROBOT_ESTIMATOR_ROBOT_EKF_HPP_

#include <Eigen/Dense>

class RobotEKF {
public:
    RobotEKF(double dt);
    ~RobotEKF() = default;

    // Core EKF Steps
    Eigen::Vector3d predict(double omega_enc);
    Eigen::Vector3d update(double omega_gyro, double omega_enc);

    // Getters
    double getHeading() const { return theta_; }
    double getCovariance() const { return P_; }

private:
    double dt_;

    // State vector: [x, y, theta]^T
    double theta_;

    // Covariance matrices
    double P_;
    double Q_;
    double R_;

    void normalizeangle();
};

#endif // ROBOT_ESTIMATOR_ROBOT_EKF_HPP_