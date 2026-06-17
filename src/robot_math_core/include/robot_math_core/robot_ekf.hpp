#ifndef ROBOT_ESTIMATOR_ROBOT_EKF_HPP_
#define ROBOT_ESTIMATOR_ROBOT_EKF_HPP_

namespace robot_math_core
{

class EkfFilter {
public:
    EkfFilter();
    ~EkfFilter() = default;

    // Core EKF Steps running with dynamic dt values
    double predict(double omega_enc, double dt);
    double update(double omega_gyro, double omega_enc, double dt);

    // Noise setup parameter
    void setNoiseParameters(double q_theta, double r_gyro);
    void resetState(double initial_theta, double initial_P);

    // Getters
    double getHeading() const { return theta_; }
    double getCovariance() const { return P_; }

private:
    double theta_;
    double P_;
    double Q_;
    double R_;
};

}  // namespace robot_math_core

#endif // ROBOT_ESTIMATOR_ROBOT_EKF_HPP_