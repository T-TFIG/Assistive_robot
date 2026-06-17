#include "robot_estimators/pure_encoder.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>

namespace robot_estimators
{

PureEncoder::PureEncoder() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn PureEncoder::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PureEncoder::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(get_node()->get_logger(), "downloading the parameter of the pure encoder from yaml ...");

    try
    {
        param_listener_ = std::make_shared<pure_encoder::ParamListener>(get_node());
        params_ = param_listener_->get_params();
        RCLCPP_INFO(get_node()->get_logger(), "download finished");
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to download parameters: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    try
    {
        auto odom_pub = get_node()->create_publisher<nav_msgs::msg::Odometry>(params_.odom_topic, rclcpp::SystemDefaultsQoS());
        rt_odom_pub_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odom_pub);
        tf_boardcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*get_node());
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize topic and connectivity: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration PureEncoder::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration PureEncoder::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names = {
        "mobile_base_Revolute_1/velocity",
        "mobile_base_Revolute_2/velocity",
        "mobile_base_Revolute_3/velocity",
        "mobile_base_Revolute_4/velocity"
  };

  return config;
}

controller_interface::CallbackReturn PureEncoder::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  odom_x_ = 0.0;
  odom_y_ = 0.0;
  odom_yaw_ = 0.0;
  return controller_interface::CallbackReturn::SUCCESS;
}

Eigen::Matrix<double, 3, 1> PureEncoder::forward_kinematic()
{
    const double r = params_.robot_dimensions.wheel_radius;
    const double L = params_.robot_dimensions.robot_length;
    const double W = params_.robot_dimensions.robot_width;

    Eigen::Matrix<double, 4, 3> M;
    M <<  -(L+W), 1, -1,
           (L+W), 1,  1,
           (L+W), 1, -1,
          -(L+W), 1,  1;

    Eigen::Matrix<double, 3, 4> M_pinv =
        (M.transpose() * M).inverse() * M.transpose();

    Eigen::Matrix<double, 4, 1> omega;
    omega << state_interfaces_[0].get_value(),
             state_interfaces_[1].get_value(),
             state_interfaces_[2].get_value(),
             state_interfaces_[3].get_value();

    return r * M_pinv * omega;
}

controller_interface::return_type PureEncoder::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  double dt = period.seconds();
  if (dt <= 0.0) return controller_interface::return_type::OK;

  Eigen::Matrix<double, 3, 1> fk = forward_kinematic();
  double wz_enc  = fk(0);
  double vel_x   = fk(1);
  double vel_y   = fk(2);

  odom_yaw_ += wz_enc * dt;
  odom_x_ += (vel_x * std::cos(odom_yaw_) - vel_y * std::sin(odom_yaw_)) * dt;
  odom_y_ += (vel_x * std::sin(odom_yaw_) + vel_y * std::cos(odom_yaw_)) * dt;

  if (rt_odom_pub_ && rt_odom_pub_->trylock())
  {
    auto & msg = rt_odom_pub_->msg_;
    msg.header.stamp = time;
    msg.header.frame_id = params_.odom_frame_id;
    msg.child_frame_id = params_.base_frame_id;

    msg.pose.pose.position.x = odom_x_;
    msg.pose.pose.position.y = odom_y_;
    msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, odom_yaw_);
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    msg.twist.twist.linear.x = vel_x;
    msg.twist.twist.linear.y = vel_y;
    msg.twist.twist.angular.z = wz_enc;

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = time;
    t.header.frame_id = params_.odom_frame_id;
    t.child_frame_id = params_.base_frame_id;
    t.transform.translation.x = odom_x_;
    t.transform.translation.y = odom_y_;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_boardcaster_->sendTransform(t);
    rt_odom_pub_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

} // namespace robot_estimators

PLUGINLIB_EXPORT_CLASS(robot_estimators::PureEncoder, controller_interface::ControllerInterface)
