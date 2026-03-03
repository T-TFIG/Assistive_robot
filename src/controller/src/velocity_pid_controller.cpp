// #include "controller/velocity_pid_controller.hpp"
#include "/home/nathaphong_meng/Documents/mater_degree_Bristol/Assistive_robot/src/controller/include/controller/velocity_pid_controller.hpp"


namespace omni_pid_controller
{
PidController::PidController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn PidController::on_init()
{
    try {
        param_listener_ = std::make_shared<pid_controller::ParamListener>(get_node()); 
        params_ = param_listener_->get_params();
        RCLCPP_INFO(get_node()->get_logger(), "Number of joints found: %zu", params_.dof_names.size());
        RCLCPP_INFO(get_node()->get_logger(), "Number of joints found: %f", params_.wheel_1.kp);
        
    } catch (const std::exception & e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Exception during parameter init: %s", e.what());
        return CallbackReturn::ERROR;
    }

    pids_.resize(params_.num_of_wheels);
    
    pids_[0] = {params_.wheel_1.kp, params_.wheel_1.ki, params_.wheel_1.kd, 0.0, 0.0, params_.i_max, params_.i_min};
    pids_[1] = {params_.wheel_2.kp, params_.wheel_2.ki, params_.wheel_2.kd, 0.0, 0.0, params_.i_max, params_.i_min};
    pids_[2] = {params_.wheel_3.kp, params_.wheel_3.ki, params_.wheel_3.kd, 0.0, 0.0, params_.i_max, params_.i_min};
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PidController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    auto callback = [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg){rt_command_ptr_.writeFromNonRT(msg);};
    rt_command_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>("~/cmd_vel", rclcpp::SystemDefaultsQoS(), callback);
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PidController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration PidController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names = {"mobile_base_Revolute_1/effort", "mobile_base_Revolute_2/effort", "mobile_base_Revolute_3/effort"};
    return config;
}

controller_interface::InterfaceConfiguration PidController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names = {"mobile_base_Revolute_1/velocity", "mobile_base_Revolute_2/velocity", "mobile_base_Revolute_3/velocity"};
    return config;
}

std::vector<double> PidController::inverse_kinematic(std::shared_ptr<geometry_msgs::msg::Twist> command)
{   
    double vx = command->linear.x;
    double vy = command->linear.y;
    double omega = command->angular.z;

    std::vector<double> command_joint(3);
    command_joint[0] = (-0.5 * vx + 0.866 * vy + params_.robot_radius * omega) / params_.wheel_radius;
    command_joint[1] = (-0.5 * vx - 0.866 * vy + params_.robot_radius * omega) / params_.wheel_radius;
    command_joint[2] = (vx + params_.robot_radius * omega) / params_.wheel_radius;

    return command_joint;
}

double PidController::compute_pid_command(double& error, double& dt, int motor_num)
{
    auto & pid = pids_[motor_num];

    pid.integral += error * dt;

    if (pid.integral >= pid.i_max) {pid.integral = pid.i_max;}
    if (pid.integral <= pid.i_min) {pid.integral = pid.i_min;}

    double derivative = (error - pid.prev_error) / dt;

    double output = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;

    pid.prev_error = error;

    return output;
}


controller_interface::return_type PidController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
    auto command = rt_command_ptr_.readFromRT();
    if (!command || !(*command)) return controller_interface::return_type::OK;

    std::vector<double> command_joint = inverse_kinematic(*command);
    
    double dt = period.seconds();

    if (dt <= 0.0)
        return controller_interface::return_type::OK;

    for (int i = 0; i < 3; i++)
    {
        double current_vel = state_interfaces_[i].get_value();
        double error = command_joint[i] - current_vel;
        
        // prevent NaN explosion
        double output = compute_pid_command(error, dt, i);
        
        RCLCPP_INFO(get_node()->get_logger(), "Inverse kinematic: %d = %f", i, output);

        // Safety Clamps
        if (output > 5.0) output = 5.0;
        if (output < -5.0) output = -5.0;
        if (std::isnan(output)) output = 0.0;

        command_interfaces_[i].set_value(output);
    }
    return controller_interface::return_type::OK;
}

} 

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(omni_pid_controller::PidController, controller_interface::ControllerInterface)