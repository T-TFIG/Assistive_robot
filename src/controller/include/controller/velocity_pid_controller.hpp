#ifndef OMNI_WHEEL_CONTROLLER_HPP_
#define OMNI_WHEEL_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "control_toolbox/pid.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "realtime_tools/realtime_buffer.hpp"

#include "pid_controller_parameters.hpp"

namespace pid_controller
{
  class PidController : public controller_interface::ChainableControllerInterface
  {
    public:
      PidController();

      /*
        the ways we write override because we trying to replace the parent file which it was the same name as we write
        but the thing here is that if the function name that we write and we doesn't have a name of this thing in the 
        parent file it the program will notice us that this function name that you trying to replace is an incorrect name
        or not real
      */
      
      // Lifecycle Functions
      controller_interface::CallbackReturn on_init() override;
      controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
      controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

      //Interface Configuration
      controller_interface::InterfaceConfiguration command_interface_configuration() const override;
      controller_interface::InterfaceConfiguration state_interface_configuration() const override;

      // The Real-Time Loop Functions
      controller_interface::return_type update_reference_from_subscribers() override;
      controller_interface::return_type update_and_write_commands(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    protected:
      //Helper function
      std::vector<double> inverse_kinematic(std::shared_ptr<geometry_msgs::msg::Twist> command);

      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr rt_command_sub_;
      realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Twist>> rt_command_ptr_;

      // PID objects for each of the 3 wheels
      std::vector<std::unique_ptr<control_toolbox::Pid>> pids_;

      // Robot parameters (Distance to wheels, etc.)
      double wheel_radius_ = 0.05; // 5cm
      double robot_radius_ = 0.15; // 15cm

      std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

      // shared pointer 
      std::shared_ptr<ParamListener> param_listener_;
      Params params_;

  };  
}

#endif