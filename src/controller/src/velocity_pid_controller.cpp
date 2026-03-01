// #include "include/controller/velocity_pid_controller.hpp"
#include "src/controller/include/controller/velocity_pid_controller.hpp"


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
    pids_.resize(3);
    for (auto & pid : pids_) {
        pid = std::make_unique<control_toolbox::Pid>();
        pid->initPid(params_., params_.ki, params_.kd, params_.i_max, params_.i_min);
    }

    return CallbackReturn::SUCCESS;
    }


    controller_interface::InterfaceConfiguration PidController::command_interface_configuration()
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        config.names.push_back("wheel_1_joint/velocity");
        config.names.push_back("wheel_2_joint/velocity");
        config.names.push_back("wheel_3_joint/velocity");
        return config;
    }

    controller_interface::InterfaceConfiguration PidController::state_interface_configuration()
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        config.names.push_back("wheel_1_joint/velocity");
        config.names.push_back("wheel_2_joint/velocity");
        config.names.push_back("wheel_3_joint/velocity");
        return config;
    }

    PidController::inverse_kinematic(auto command)
    {   
        double vx = (*command)->linear.x;
        double vy = (*command)->linear.y;
        double omega = (*command)->angular.z;

        std::vector<double> command_joint;
        command_joint[0] = (-0.5 * vx + 0.866 * vy + R * omega) / params_.wheel_radius;
        command_joint[1] = (-0.5 * vx - 0.866 * vy + R * omega) / params_.wheel_radius;
        command_joint[2] = (vx + R * omega) / params_.wheel_radius; 

        return command_joint;
    }

    controller_interface::return_type PidController::update_and_write_commands(
        const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        auto command = rt_command_sub_.get();
        std::vector<double> command_joint = inverse_kinematic(command);

        for (int i = 0; i < 3; i++)
        {
            double current_vel = state_interface_[i].get_value();
            double error = command_joint[i] - current_vel;

            double output = pids_[i]->computeCommand(error, period.nanoseconds());

            command_interfaces_[i].set_value((output));
        }
        
        return controller_interface::return_type::OK;
    }



}