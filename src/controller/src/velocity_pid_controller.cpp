// #include "/home/nathaphong_meng/Documents/mater_degree_Bristol/Assistive_robot/src/controller/include/controller/velocity_pid_controller.hpp"
#include "controller/velocity_pid_controller.hpp"

namespace pid_controller
{
PidController::PidController() : controller_interface::ChainableControllerInterface() {}

    controller_interface::CallbackReturn PidController::on_init()
    {
    try {
        // Create the listener
        param_listener_ = std::make_shared<ParamListener>(get_node());
        // Fetch initial parameters
        params_ = param_listener_->get_params();
    } catch (const std::exception & e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Exception during parameter init: %s", e.what());
        return CallbackReturn::ERROR;
    }

    // Initialize your 3 PID objects
    pids_.resize(params_.num_of_wheels);
    for (int i = 0; i < params_.num_of_wheels; i++)
    {
        pids_[i] = std::make_unique<control_toolbox::Pid>();
    }
    
    pids_[0]->initPid(params_.wheel_1.kp, params_.wheel_1.ki, params_.wheel_1.kd, params_.i_max, params_.i_min);
    pids_[1]->initPid(params_.wheel_2.kp, params_.wheel_2.ki, params_.wheel_2.kd, params_.i_max, params_.i_min);
    pids_[2]->initPid(params_.wheel_3.kp, params_.wheel_3.ki, params_.wheel_3.kd, params_.i_max, params_.i_min);

    return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PidController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        auto callback = [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg){rt_command_ptr_.writeFromNonRT(msg);};
        rt_command_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>("~/cmd_vel", rclcpp::SystemDefaultsQoS(), callback);

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PidController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }


    controller_interface::InterfaceConfiguration PidController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        config.names = {"mobile_base_Revolute-1/effort", 
                        "mobile_base_Revolute-2/effort", 
                        "mobile_base_Revolute-3/effort"};
        return config;
    }

    controller_interface::InterfaceConfiguration PidController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        config.names = {"mobile_base_Revolute-1/velocity", 
                        "mobile_base_Revolute-2/velocity", 
                        "mobile_base_Revolute-3/velocity"};
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

    controller_interface::return_type PidController::update_reference_from_subscribers()
    {
        return controller_interface::return_type::OK;
    }

    controller_interface::return_type PidController::update_and_write_commands(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
    {
        auto command = rt_command_ptr_.readFromRT();
        
        if (!command || !(*command))
        {
            return controller_interface::return_type::OK;
        }

        std::vector<double> command_joint = inverse_kinematic(*command);

        for (int i = 0; i < 3; i++)
        {
            double current_vel = state_interfaces_[i].get_value();
            double error = command_joint[i] - current_vel;
            double output = pids_[i]->computeCommand(error, period.nanoseconds());
            command_interfaces_[i].set_value((output));
        }
        
        return controller_interface::return_type::OK;
    }

    std::vector<hardware_interface::CommandInterface> PidController::on_export_reference_interfaces()
    {
        return {};
    }

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(pid_controller::PidController, controller_interface::ChainableControllerInterface)