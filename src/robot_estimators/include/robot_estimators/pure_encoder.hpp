#ifndef ROBOT_ESTIMATORS__PURE_ENCODER_HPP_
#define ROBOT_ESTIMATORS__PURE_ENCODER_HPP_

#include <memory>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>

#include "controller_interface/controller_interface.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "realtime_tools/realtime_publisher.hpp"
#include <robot_estimators/pure_encoder_yaml.hpp>

namespace robot_estimators
{

    class PureEncoder : public controller_interface::ControllerInterface
    {
        public:
            PureEncoder();

            controller_interface::CallbackReturn on_init() override;
            controller_interface::InterfaceConfiguration command_interface_configuration() const override;
            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::return_type update(
                const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:
            Eigen::Matrix<double, 3, 1> forward_kinematic();

            double odom_x_ = 0.0;
            double odom_y_ = 0.0;
            double odom_yaw_ = 0.0;

            std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> rt_odom_pub_;
            std::unique_ptr<tf2_ros::TransformBroadcaster> tf_boardcaster_;

            std::shared_ptr<pure_encoder::ParamListener> param_listener_;
            pure_encoder::Params params_;
    };

} // namespace robot_estimators

#endif // ROBOT_ESTIMATORS__PURE_ENCODER_HPP_
