#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include ""
#include "nav_msgs/msg/odometry.hpp" // odom val
#include "sensor_msgs/msg/imu.hpp"   // IMU val

using namespace std::chrono_literals;

class Robot_EKF : public rclcpp::Node
{
  public:
    Robot_EKF()
    : Node("Robot_EKF")
    {
      Odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&Robot_EKF::Odom_callback, this, _1));
      IMU_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>("imu_sensor_broadcaster/imu", 10);
      
      EKF_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_EKF", 10);
    }

  private:
    void Odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
    {

    }
    rclcpp::TimerBase::SharedPtr timer_;

    // Input sensor
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr IMU_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr Odom_subscriber_; 

    // State estimator output
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr EKF_publisher_;

    // 
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}