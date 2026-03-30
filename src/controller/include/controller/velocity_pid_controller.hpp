#ifndef OMNI_WHEEL_CONTROLLER_HPP_
#define OMNI_WHEEL_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "realtime_tools/realtime_buffer.hpp"

#include <controller/pid_controller_parameters.hpp>

namespace omni_pid_controller
{

class PidController : public controller_interface::ControllerInterface
{
public:
  PidController();

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

protected:


  struct SimplePID
  {
    double kp;
    double ki;
    double kd;

    double integral = 0.0;
    double prev_error = 0.0;

    double i_max;
    double i_min;
  };



  Eigen::Matrix<double, 4, 1> inverse_kinematic(const geometry_msgs::msg::Twist &cmd);

  Eigen::Matrix<double, 3, 1> forward_kinematic();


  void Odometry(const Eigen::Matrix<double, 3, 1>& vel, double dt);


  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr rt_command_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_boardcaster_;


  realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Twist>> rt_command_ptr_;
  realtime_tools::RealtimeBuffer<Eigen::Matrix<double, 3, 1>> rt_twist_buffer_;


  std::vector<SimplePID> pids_;
  double compute_pid_command(double& error, double& dt, int motor_num);


  std::shared_ptr<pid_controller::ParamListener> param_listener_;
  pid_controller::Params params_;
  nav_msgs::msg::Odometry odom_message;
  geometry_msgs::msg::TransformStamped tf;
  tf2::Quaternion q;

  std::vector<double> pseudo_odom_;

  double debugging;
};

} // namespace omni_pid_controller

#endif