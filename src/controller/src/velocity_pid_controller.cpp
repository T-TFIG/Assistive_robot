#include "controller/velocity_pid_controller.hpp"

namespace omni_pid_controller
{

PidController::PidController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn PidController::on_init()
{
    auto result = get_node()->set_parameter(rclcpp::Parameter("use_sim_time", true));
    (void)result;

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

    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PidController::on_activate(const rclcpp_lifecycle::State &)
{
    for (auto & pid : pids_) {
        pid.integral = 0.0;
        pid.prev_error = 0.0;
    }
    return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration PidController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    config.names = {
        "mobile_base_Revolute_1/effort",
        "mobile_base_Revolute_2/effort",
        "mobile_base_Revolute_3/effort",
        "mobile_base_Revolute_4/effort"
    };

    return config;
}

controller_interface::InterfaceConfiguration PidController::state_interface_configuration() const
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

Eigen::Matrix<double, 4, 1> PidController::inverse_kinematic(const geometry_msgs::msg::Twist &cmd)
{
    const double r = params_.wheel_radius;
    const double L = params_.robot_length;
    const double W = params_.robot_width;

    Eigen::Matrix<double, 4, 3> M;
    M <<  -(L+W), 1, -1,
           (L+W), 1,  1,
           (L+W), 1, -1,
          -(L+W), 1,  1;

    Eigen::Matrix<double, 3, 1> v;
    v << cmd.angular.z,
         cmd.linear.x,
         cmd.linear.y;

    return (1.0 / r) * M * v;
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
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & period)
{
    auto command = rt_command_ptr_.readFromRT();

    geometry_msgs::msg::Twist cmd_vel;
    if (command && *command)
        cmd_vel = **command;

    double dt = period.seconds();
    if (dt <= 0.0) return controller_interface::return_type::OK;

    Eigen::Matrix<double, 4, 1> desired = inverse_kinematic(cmd_vel);

    for (int i = 0; i < 4; i++)
    {
        double current = state_interfaces_[i].get_value();
        double error = desired(i) - current;

        double effort = compute_pid_command(error, dt, i);

        if (effort > 10.0) effort = 10.0;
        if (effort < -10.0) effort = -10.0;
        if (std::isnan(effort)) effort = 0.0;

        command_interfaces_[i].set_value(effort);
    }

    return controller_interface::return_type::OK;
}

} // namespace omni_pid_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(omni_pid_controller::PidController, controller_interface::ControllerInterface)
