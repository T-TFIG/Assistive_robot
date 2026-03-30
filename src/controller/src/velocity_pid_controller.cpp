#include "controller/velocity_pid_controller.hpp"

namespace omni_pid_controller
{

PidController::PidController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn PidController::on_init()
{
    auto result = get_node()->set_parameter(rclcpp::Parameter("use_sim_time", true));

    try {
        param_listener_ = std::make_shared<pid_controller::ParamListener>(get_node()); 
        params_ = param_listener_->get_params();

        RCLCPP_INFO(get_node()->get_logger(), "Number of joints: %zu", params_.dof_names.size());
    } catch (const std::exception & e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Param init failed: %s", e.what());
        return CallbackReturn::ERROR;
    }

    pids_.resize(4);

    pids_[0] = {params_.wheel_1.kp, params_.wheel_1.ki, params_.wheel_1.kd, 0.0, 0.0, params_.i_max, params_.i_min};
    pids_[1] = {params_.wheel_2.kp, params_.wheel_2.ki, params_.wheel_2.kd, 0.0, 0.0, params_.i_max, params_.i_min};
    pids_[2] = {params_.wheel_3.kp, params_.wheel_3.ki, params_.wheel_3.kd, 0.0, 0.0, params_.i_max, params_.i_min};
    pids_[3] = {params_.wheel_4.kp, params_.wheel_4.ki, params_.wheel_4.kd, 0.0, 0.0, params_.i_max, params_.i_min};

    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PidController::on_configure(const rclcpp_lifecycle::State &)
{
    auto callback = [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg){
        rt_command_ptr_.writeFromNonRT(msg);
    };

    rt_command_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
        "~/cmd_vel", rclcpp::SystemDefaultsQoS(), callback);

    odom_pub_ = get_node()->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::SystemDefaultsQoS());

    tf_boardcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*get_node());

    pseudo_odom_.resize(5, 0.0);

    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PidController::on_activate(const rclcpp_lifecycle::State &)
{
    return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration PidController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    // ⚠️ MUST match URDF joint names
    config.names = {
        "wheel_1_joint/effort",
        "wheel_2_joint/effort",
        "wheel_3_joint/effort",
        "wheel_4_joint/effort"
    };

    return config;
}

controller_interface::InterfaceConfiguration PidController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    config.names = {
        "wheel_1_joint/velocity",
        "wheel_2_joint/velocity",
        "wheel_3_joint/velocity",
        "wheel_4_joint/velocity"
    };

    return config;
}

Eigen::Matrix<double, 4, 1> PidController::inverse_kinematic(const geometry_msgs::msg::Twist &cmd)
{
    const double r = params_.wheel_radius;
    const double L = params_.robot_length / 2.0;
    const double W = params_.robot_width / 2.0;

    double k = L + W;

    Eigen::Matrix<double, 4, 3> M;

    M <<  1, -1, -k,
          1,  1,  k,
          1,  1, -k,
          1, -1,  k;

    Eigen::Matrix<double, 3, 1> v;
    v << cmd.linear.x,
         cmd.linear.y,
         cmd.angular.z;

    return (1.0 / r) * M * v;
}

Eigen::Matrix<double, 3, 1> PidController::forward_kinematic()
{
    const double r = params_.wheel_radius;
    const double L = params_.robot_length / 2.0;
    const double W = params_.robot_width / 2.0;

    double k = L + W;

    Eigen::Matrix<double, 4, 3> M;

    M <<  1, -1, -k,
          1,  1,  k,
          1,  1, -k,
          1, -1,  k;

    Eigen::Matrix<double, 3, 4> M_pinv =
        (M.transpose() * M).inverse() * M.transpose();

    Eigen::Matrix<double, 4, 1> omega;
    omega << state_interfaces_[0].get_value(),
             state_interfaces_[1].get_value(),
             state_interfaces_[2].get_value(),
             state_interfaces_[3].get_value();

    return r * M_pinv * omega;
}

double PidController::compute_pid_command(double& error, double& dt, int i)
{
    auto & pid = pids_[i];

    pid.integral += error * dt;

    if (pid.integral > pid.i_max) pid.integral = pid.i_max;
    if (pid.integral < pid.i_min) pid.integral = pid.i_min;

    double derivative = (error - pid.prev_error) / dt;

    double output = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;

    pid.prev_error = error;

    return output;
}

controller_interface::return_type PidController::update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)
{
    auto command = rt_command_ptr_.readFromRT();

    geometry_msgs::msg::Twist cmd_vel;

    if (command && *command)
        cmd_vel = **command;

    Eigen::Matrix<double, 4, 1> desired = inverse_kinematic(cmd_vel);

    double dt = period.seconds();
    if (dt <= 0.0) return controller_interface::return_type::OK;

    for (int i = 0; i < 4; i++)
    {
        double current = state_interfaces_[i].get_value();
        double error = desired(i) - current;

        double effort = compute_pid_command(error, dt, i);

        // safety clamp
        if (effort > 10.0) effort = 10.0;
        if (effort < -10.0) effort = -10.0;
        if (std::isnan(effort)) effort = 0.0;

        command_interfaces_[i].set_value(effort);
    }

    // odometry
    auto vel = forward_kinematic();

    double theta = pseudo_odom_[2];

    pseudo_odom_[0] += (vel(0) * cos(theta) - vel(1) * sin(theta)) * dt;
    pseudo_odom_[1] += (vel(0) * sin(theta) + vel(1) * cos(theta)) * dt;
    pseudo_odom_[2] += vel(2) * dt;

    return controller_interface::return_type::OK;
}

} // namespace omni_pid_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(omni_pid_controller::PidController, controller_interface::ControllerInterface)