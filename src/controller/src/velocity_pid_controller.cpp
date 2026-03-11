// #include "controller/velocity_pid_controller.hpp"
#include "/home/nathaphong_meng/Documents/mater_degree_Bristol/Assistive_robot/src/controller/include/controller/velocity_pid_controller.hpp"


namespace omni_pid_controller
{
PidController::PidController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn PidController::on_init()
{
    auto result = get_node()->set_parameter(rclcpp::Parameter("use_sim_time", true));

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
    odom_pub_ = get_node()->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::SystemDefaultsQoS());
    // timer_ = get_node()->create_timer(std::chrono::milliseconds(10), std::bind(&PidController::timer_callback, this));
    tf_boardcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*get_node());
    pseudo_odom_.resize(5, 0.0);
    debugging = 0.0;
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

Eigen::Matrix<double, 3 ,1> PidController::forward_kinematic()
{
    const double r = params_.wheel_radius;
    const double R = params_.robot_radius;

    // The projection of wheel velocities onto Robot X, Y, and Theta
    Eigen::Matrix<double, 3, 3> M;
    M <<  cos(0.0),   cos(2*M_PI/3),   cos(4*M_PI/3), // Projections on X
         -sin(0.0),  -sin(2*M_PI/3),  -sin(4*M_PI/3), // Projections on Y
          1.0/R,       1.0/R,            1.0/R;         // Contribution to Yaw

    Eigen::Matrix<double, 3, 1> omega;
    // Your specific 2-0-1 mapping
    omega << state_interfaces_[2].get_value(),
             state_interfaces_[0].get_value(),
             state_interfaces_[1].get_value();

    Eigen::Matrix<double, 3, 1> vel_body = M * omega;

    // Apply the correct kinematic scaling
    vel_body(0) *= (r * (2.0/3.0)); 
    vel_body(1) *= (r * (2.0/3.0));
    vel_body(2) *= (r / 3.0); 

    return vel_body;
}

std::vector<double> PidController::inverse_kinematic(const geometry_msgs::msg::Twist &command)
{   
    double vx = command.linear.x;
    double vy = command.linear.y;
    double omega = command.angular.z;

    std::vector<double> command_joint(3);
    command_joint[0] = (-0.5 * vx + 0.866 * vy + params_.robot_radius * omega) / params_.wheel_radius;
    command_joint[1] = (-0.5 * vx - 0.866 * vy + params_.robot_radius * omega) / params_.wheel_radius;
    command_joint[2] = (vx + params_.robot_radius * omega) / params_.wheel_radius;

    return command_joint;
}

void PidController::Odometry(auto vel, double dt)
{
    // remember that the odom itself is the fixed frame treat like a global frame and base_link is its child 
    double theta = pseudo_odom_[2];

    pseudo_odom_[0] += (vel(0) * cos(theta) - vel(1) * sin(theta)) * dt;        // vel_x
    pseudo_odom_[1] += (vel(0) * sin(theta) + vel(1) * cos(theta)) * dt;        // vel_y
    pseudo_odom_[2] -= vel(2) * dt;     // angular pos

    pseudo_odom_[3] = vel(0);       // linear velocity x 
    pseudo_odom_[4] = -vel(2);       // angular velocity yaw
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


controller_interface::return_type PidController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    auto command = rt_command_ptr_.readFromRT();
    // rclcpp::Time now = get_node()->get_clock()->now();

    geometry_msgs::msg::Twist cmd_vel;

    if (command && *command)
    {
        cmd_vel = **command;
    }
    else
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
    }
    
    std::vector<double> command_joint = inverse_kinematic(cmd_vel);
    
    double dt = period.seconds();

    if (dt <= 0.0)
        return controller_interface::return_type::OK;

    if (params_.header_frame.empty() || params_.child_frame.empty()) {
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000, 
                             "Waiting for parameters... Frames are still empty!");
        return controller_interface::return_type::OK;
    }


    for (int i = 0; i < 3; i++)
    {
        double current_vel = state_interfaces_[i].get_value();
        double error = command_joint[i] - current_vel;
        
        // prevent NaN explosion
        double output = compute_pid_command(error, dt, i);
        // Safety Clamps
        if (output > 5.0) output = 5.0;
        if (output < -5.0) output = -5.0;
        if (std::isnan(output)) output = 0.0;

        command_interfaces_[i].set_value(output);
    }

    // compute all the kinematic and odometry
    auto twist = PidController::forward_kinematic();


    PidController::Odometry(twist, dt);
    rt_twist_buffer_.writeFromNonRT(twist);

    // compute quaternion
    q.setRPY(0, 0, pseudo_odom_[2] + params_.init_odom_val.Yaw);
    geometry_msgs::msg::Quaternion q_msg = tf2::toMsg(q);


    // Odom message 
    odom_message.header.stamp = time;
    odom_message.header.frame_id = params_.header_frame;
    odom_message.child_frame_id = params_.child_frame;

    odom_message.pose.pose.position.x = pseudo_odom_[0] + params_.init_odom_val.X;
    odom_message.pose.pose.position.y = pseudo_odom_[1] + params_.init_odom_val.Y;
    odom_message.pose.pose.position.z = 0.0;

    odom_message.pose.pose.orientation = q_msg;

    odom_message.twist.twist.linear.x = pseudo_odom_[3];
    odom_message.twist.twist.angular.z = pseudo_odom_[4];

    // tf message
    tf.header.stamp = time;
    tf.header.frame_id = params_.header_frame;
    tf.child_frame_id = params_.child_frame;

    tf.transform.translation.x = pseudo_odom_[0] + params_.init_odom_val.X;
    tf.transform.translation.y = pseudo_odom_[1] + params_.init_odom_val.Y;
    tf.transform.translation.z = 0.0;

    tf.transform.rotation = q_msg;
    
    odom_pub_->publish(odom_message);
    tf_boardcaster_->sendTransform(tf);
    // rt_odom_buffer_.writeFromNonRT(odom_message);
    // rt_tf_boardcast_buffer_.writeFromNonRT(tf);

    return controller_interface::return_type::OK;
}

} 

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(omni_pid_controller::PidController, controller_interface::ControllerInterface)