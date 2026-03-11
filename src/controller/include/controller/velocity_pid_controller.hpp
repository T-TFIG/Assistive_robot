#ifndef OMNI_WHEEL_CONTROLLER_HPP_
#define OMNI_WHEEL_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <math.h>
#include <Eigen/Dense>

#include "control_toolbox/pid.hpp"
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
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

    std::vector<double> inverse_kinematic(const geometry_msgs::msg::Twist &command);
    void Odometry(auto vel, double dt);
    Eigen::Matrix<double, 3, 1> forward_kinematic();

    // declared ros2 message parameters (sub and pub) and its tf boardcast
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr rt_command_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_boardcaster_;

    // Timer for publishing
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;

    // lovely realtime tools (my toxic relationship => function with benefit)
    realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Twist>> rt_command_ptr_;
    realtime_tools::RealtimeBuffer<Eigen::Matrix<double,3,1>> rt_twist_buffer_;
    realtime_tools::RealtimeBuffer<nav_msgs::msg::Odometry> rt_odom_buffer_;
    realtime_tools::RealtimeBuffer<geometry_msgs::msg::TransformStamped> rt_tf_boardcast_buffer_;

    // dedicated controller (PID) (once i used to make it from controller_toolbox)
    std::vector<SimplePID> pids_;
    double compute_pid_command(double& error, double& dt, int motor_num);
    
    // yaml pulling variable for more easier to adjust parameter
    std::shared_ptr<pid_controller::ParamListener> param_listener_;  // add namespace from shitty controller.yaml
    pid_controller::Params params_;                                  // add namespace
    
    
    // odometry message 
    nav_msgs::msg::Odometry odom_message;
    geometry_msgs::msg::TransformStamped tf;
    tf2::Quaternion q;
    std::vector<double> pseudo_odom_;   // i am only keep the value of x y and yaw inside this vector

    //debugging
    double debugging;


  };
}
#endif