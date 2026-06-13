#include <chrono>
#include <functional>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "Kalman_Filter/robot_ekf.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class EKFNode : public rclcpp::Node
{
public:
    EKFNode()
    : Node("ekf_node"), x_(0.0), y_(0.0),
      odom_vx_(0.0), odom_vy_(0.0), odom_wz_(0.0),
      imu_wz_(0.0), imu_received_(false), fused_theta_global_(0.0)
    {
        // Notice we don't pass a fixed dt to the constructor anymore
        ekf_ = std::make_unique<RobotEKF>();

        Odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&EKFNode::odom_callback, this, _1));
        IMU_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu_sensor_broadcaster/imu", 10, std::bind(&EKFNode::imu_callback, this, _1));

        Odom_EKF_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_EKF", 10);

        // Initialize the tracking timestamp for the timer loop
        last_timer_time_ = this->get_clock()->now();

        timer_ = this->create_wall_timer(10ms, std::bind(&EKFNode::timer_callback, this));
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        rclcpp::Time current_time = msg->header.stamp;
        
        // Save raw incoming data targets
        odom_vx_ = msg->twist.twist.linear.x;
        odom_vy_ = msg->twist.twist.linear.y;   
        odom_wz_ = msg->twist.twist.angular.z;

        // Initialize time on the first callback frame
        if (last_odom_time_.seconds() == 0.0) {
            last_odom_time_ = current_time;
            return;
        }

        // Calculate actual time delta between wheel ticks (~0.026s for 38Hz)
        double dt = (current_time - last_odom_time_).seconds();
        last_odom_time_ = current_time;

        // Dynamic position integration bound perfectly to physical encoder data updates
        x_ += odom_vx_ * std::cos(fused_theta_global_) * dt;
        y_ += odom_vx_ * std::sin(fused_theta_global_) * dt;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        imu_wz_       = msg->angular_velocity.z;
        imu_received_ = true;
    }

    void timer_callback()
    {
        rclcpp::Time current_time = this->get_clock()->now();
        
        // Calculate the dynamic dt for the EKF update loop
        double dt = (current_time - last_timer_time_).seconds();
        last_timer_time_ = current_time;

        // Guard against an uninitialized or zero-bound initial clock step
        if (dt <= 0.0) {
            return;
        }

        // 1. Prediction step using dynamic dt
        ekf_->predict(odom_wz_, dt);

        // 2. Correction step using dynamic dt
        if (imu_received_) {
            fused_theta_global_ = ekf_->update(imu_wz_, odom_wz_, dt);
        } else {
            fused_theta_global_ = ekf_->getHeading();
        }

        // Build and publish message
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp    = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id  = "base_link";

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = std::sin(fused_theta_global_ * 0.5);
        odom_msg.pose.pose.orientation.w = std::cos(fused_theta_global_ * 0.5);

        odom_msg.pose.covariance[35] = ekf_->getCovariance();

        odom_msg.twist.twist.linear.x  = odom_vx_;
        odom_msg.twist.twist.linear.y  = odom_vy_;
        odom_msg.twist.twist.angular.z = imu_wz_;

        Odom_EKF_publisher_->publish(odom_msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr   IMU_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr Odom_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr    Odom_EKF_publisher_;

    std::unique_ptr<RobotEKF> ekf_;

    double x_, y_;
    double odom_vx_, odom_vy_, odom_wz_;
    double imu_wz_;
    bool   imu_received_;
    
    double fused_theta_global_;
    rclcpp::Time last_odom_time_;
    
    // Track the precise time elapsed between consecutive timer ticks
    rclcpp::Time last_timer_time_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EKFNode>());
    rclcpp::shutdown();
    return 0;
}