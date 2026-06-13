#include "robot_estimators/ekf_estimator.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace robot_estimators
{

EkfEstimator::EkfEstimator() : controller_interface::ChainableControllerInterface() {}

controller_interface::CallbackReturn EkfEstimator::on_init()
{
  filter_core_ = std::make_unique<EkfFilter>();
  return controller_interface::CallbackReturn::SUCCESS;
}



controller_interface::CallbackReturn EkfEstimator::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    // Parameter from yaml 

    RCLCPP_INFO(get_node()->get_logger(), "downloading the parameter of the EKF from yaml ...");

    try
    {
        param_listener_ = std::make_unique<ekf_estimator::ParamListener>(get_node());
        params_ = param_listener_->get_params();

        filter_core->setNoiseParamters(
            params_.ekf_tuning.process_noise_theta,
            params_.ekf_tuning.measure_noise_gyro
        );
        
        RCLCPP_INFO(get_node()->get_logger(), "download finished");
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to download parameters: %s", e.what());
        return controller_interface::CallbackReture::ERROR;
    }
    
    // topic service or action configure 

    RCLCPP_INFO(get_node()-get_logger(), "initialize topic and connection ...");

    try
    {
        auto odom_pub_ = get_node()->create_publsiher<nav_msgs::msg::Odometry>("/odom", rclcpp::SystemDefaultsQos());

        rt_odom_pub_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odom_pub_);

        tf_boardcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*get_node());
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize topic and connectivity: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}



controller_interface::InterfaceConfiguration EkfEstimator::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
}




controller_interface::InterfaceConfiguration EkfEstimator::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  config.names = {
        "mobile_base_Revolute_1/velocity",
        "mobile_base_Revolute_2/velocity",
        "mobile_base_Revolute_3/velocity",
        "mobile_base_Revolute_4/velocity",
        "imu_sensor/angular_velocity.z"
  };
  
  return config;
}



std::vector<hardware_interface::CommandInterface> EkfEstimator::on_export_reference_interfaces()
{
  return {}; // No inputs precede the top-level estimator
}



std::vector<hardware_interface::StateInterface> EkfEstimator::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> exported_states;

  // We assign a prefix handle ("ekf_estimator") and sub-interface keys.
  // Any controller can claim these via: "ekf_estimator/fused_x", etc.
  exported_states.emplace_back(hardware_interface::StateInterface("ekf_estimator", "fused_x", &fused_x_));
  exported_states.emplace_back(hardware_interface::StateInterface("ekf_estimator", "fused_y", &fused_y_));
  exported_states.emplace_back(hardware_interface::StateInterface("ekf_estimator", "fused_yaw", &fused_yaw_));

  return exported_states;
}



controller_interface::CallbackReturn EkfEstimator::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Handle warm start positioning resets if your robot spawns at a non-zero origin
  filter_core_->resetState(0.0, 0.0, 0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

Eigen::Matrix<double, 3, 1> EkfEstimator::forward_kinematic()
{
    const double r = params_.robot_dimensions.wheel_radius;
    const double L = params_.robot_dimensions.robot_length;
    const double W = params_.robot_dimensions.robot_width;

    double k = L + W;

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


controller_interface::return_type EkfEstimator::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  double dt = period.seconds();
  if (dt <= 0.0) return controller_interface::return_type::OK;

  // 1. Safely extract raw double readings directly out of shared memory arrays
  double wz_gyro = state_interfaces_[4].get_value();

  Eigen::Matrix<double, 3, 1> FK_output_encoder = forward_kinematic();
  double vel_x = FK_output_encoder(0);
  double vel_y = FK_output_encoder(1);
  double wz_enc = FK_output_encoder(2);


  filter_core_->predict(wz_enc, dt);
  fused_yaw_ = filter_core_->update(wz_gyro, wz_enc, dt);
  

  fused_x += (vel_x * std::cos(fused_yaw_) - vel_y * std::sin(fused_yaw_)) * dt;
  fused_y += (vel_x * std::sin(fused_yaw_) - vel_y * std::cos(fused_yaw_)) * dt;

  // odometry 
  if (rt_odom_pub_ && rt_odom_pub_->trylock())
  {
    auto & odom_msg = rt_odom_pub_->msg_;
    odom_msg->header.stamp = time;
    odom_msg->header.frame_id = "odom";
    odom_msg->child_frame_id = "base_footprint";

    odom_msg->pose.pose.position.x = fused_x;
    odom_msg->pose.pose.position.y = fused_y;
    odom_msg->pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, fused_yaw_);
    odom_msg->pose.pose.orientation.x = q.x();
    odom_msg->pose.pose.orientation.y = q.y();
    odom_msg->pose.pose.orientation.z = q.z();
    odom_msg->pose.pose.orientation.w = q.w();

    odom_msg->twist.twist.linear.x = vel_x;
    odom_msg->twist.twist.linear.y = vel_y;
    odom_msg->twist.twist.angular.z = wz_gyro;

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = time;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_footprint";

    t.transform.translation.x = pseudo_odom_[0];
    t.transform.translation.y = pseudo_odom_[1];
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

// Register as a valid controller plugin
PLUGINLIB_EXPORT_CLASS(robot_estimators::EkfEstimator, controller_interface::ControllerInterface)