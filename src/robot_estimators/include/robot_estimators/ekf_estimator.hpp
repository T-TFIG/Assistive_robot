#ifndef ROBOT_ESTIMATORS__EKF_ESTIMATOR_HPP_
#define ROBOT_ESTIMATORS__EKF_ESTIMATOR_HPP_

#include <memory>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>

#include "controller_interface/chainable_controller_interface.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "robot_math_core/ekf_filter.hpp" 

namespace robot_estimators
{

    class EkfEstimator : public controller_interface::ChainableControllerInterface
    {
        public:
            EkfEstimator();

            // Core Lifecycle States
            controller_interface::CallbackReturn on_init() override;
            controller_interface::InterfaceConfiguration command_interface_configuration() const override;
            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

        protected:

            // 1. 
            std::vector<hardware_interface::CommandInterface> on_export_reference_interface() override;
            
            std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override;

            // 3. This executes your filter equations on every hard real-time hardware clock cycle.
            controller_interface::return_type update_and_write_commands(
                const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:
            // Pointer to your pure first-principles math instance
            std::unique_ptr<EkfFilter> filter_core_;

            // Thread-safe parameters or configuration names
            std::string imu_interface_name_;
            std::vector<std::string> encoder_interface_names_;

            Eigen::Matrix<double, 3, 1> forward_kinematic();

            // CRITICAL MEMORY MANAGEMENT: These variables MUST be persistent members of this class.
            // Downstream controllers will hold raw memory pointers directly to these addresses!
            double fused_x_ = 0.0;
            double fused_y_ = 0.0;
            double fused_yaw_ = 0.0;

            // Robot hardware configure
            double wheel_radius_, robot_length_, robot_width_;

            // odom publisher
            std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> rt_odom_pub_;
            std::unique_ptr<tf2_ros::TransformBroadcaster> tf_boardcaster_;

            // shared parameter part
            std::shared_ptr<ekf_estimator::ParamListener> param_listener_;
            ekf_estimator::Params params_;
    };

} // namespace robot_estimators

#endif // ROBOT_ESTIMATORS__EKF_ESTIMATOR_HPP_