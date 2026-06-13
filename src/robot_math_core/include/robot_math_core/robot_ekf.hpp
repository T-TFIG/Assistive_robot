#ifndef ROBOT_ESTIMATOR_ROBOT_EKF_HPP_
#define ROBOT_ESTIMATOR_ROBOT_EKF_HPP_

class RobotEKF {
public:
    // FIX: Default constructor takes no arguments now
    RobotEKF();
    ~RobotEKF() = default;

    // Core EKF Steps running with dynamic dt values
    double predict(double omega_enc, double dt);
    double update(double omega_gyro, double omega_enc, double dt);

    // Noise setup paramter 
    void setNoiseParameters(double q_theta, double r_gyro);
    void resetState(double initial_theta, double initial_P);

    // Getters
    double getHeading() const { return theta_; }
    double getCovariance() const { return P_; }

private:
    // State vector: heading parameter
    double theta_;

    // Covariance matrices
    double P_;
    double Q_;
    double R_;
};

#endif // ROBOT_ESTIMATOR_ROBOT_EKF_HPP_