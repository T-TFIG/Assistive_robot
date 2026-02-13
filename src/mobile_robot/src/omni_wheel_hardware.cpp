#include <chrono>
#include <vector>
#include <string>
#include <memory>
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace mobile_robot
{

class OmniWheelHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo & info) override
  {
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
      return hardware_interface::CallbackReturn::ERROR;

    // Initialize joint storage
    num_joints_ = info.joints.size();
    positions_.resize(num_joints_, 0.0);
    velocities_.resize(num_joints_, 0.0);
    commands_.resize(num_joints_, 0.0);

    RCLCPP_INFO(rclcpp::get_logger("OmniWheelHardware"), "Initialized %zu joints", num_joints_);
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Expose command and state interfaces
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < num_joints_; i++) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.joints[i].name, "position", &positions_[i]));
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.joints[i].name, "velocity", &velocities_[i]));
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < num_joints_; i++) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.joints[i].name, "velocity", &commands_[i]));
    }
    return command_interfaces;
  }

  // Read the actual robot state (simulation or real hardware)
  hardware_interface::return_type read() override
  {
    // For simulation, just copy commands to velocities (replace with your sensor reads)
    for (size_t i = 0; i < num_joints_; i++) {
      velocities_[i] = commands_[i];
      positions_[i] += velocities_[i] * 0.01; // simulate position
    }
    return hardware_interface::return_type::OK;
  }

  // Write commands to robot (simulation or real hardware)
  hardware_interface::return_type write() override
  {
    // For real robot, send commands_[i] to motor drivers
    // Example:
    // motor_driver.set_velocity(joint_name[i], commands_[i]);
    return hardware_interface::return_type::OK;
  }

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    RCLCPP_INFO(rclcpp::get_logger("OmniWheelHardware"), "Hardware activated");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    RCLCPP_INFO(rclcpp::get_logger("OmniWheelHardware"), "Hardware deactivated");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

private:
  size_t num_joints_;
  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> commands_;
};

} // namespace your_package

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(your_package::OmniWheelHardware, hardware_interface::SystemInterface)
