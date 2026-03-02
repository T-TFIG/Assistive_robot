#ifndef OMNI_WHEEL_CONTROLLER_HPP_
#define OMNI_WHEEL_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "control_toolbox/pid.hpp"
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include <controller/pid_controller_parameters.hpp>  // fix deprecated include

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
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  protected:
    std::vector<double> inverse_kinematic(std::shared_ptr<geometry_msgs::msg::Twist> command);

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr rt_command_sub_;
    realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Twist>> rt_command_ptr_;
    std::vector<std::unique_ptr<control_toolbox::Pid>> pids_;

    std::shared_ptr<pid_controller::ParamListener> param_listener_;  // add namespace from shitty controller.yaml
    pid_controller::Params params_;                                   // add namespace
  };
}
#endif