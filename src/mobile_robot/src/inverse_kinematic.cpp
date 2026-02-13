#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"     // pub
#include "geometry_msgs/msg/twist.hpp"              // sub
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */


class Inverse_kinematic : public rclcpp::Node
{
  public:
    Inverse_kinematic()
    : Node("Inverse_Kinematic")
    {
        this->declare_parameter("wheel_radius", 0.06);
        this->declare_parameter("wheel_distance", 0.1766);

        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        wheel_distance_ = this->get_parameter("wheel_distance").as_double();

        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "wheel_velocity_controller/commands", 10);
        
        subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, 
            std::bind(&Inverse_kinematic::topic_callback, this, _1)); 
        
        command_wheel_ = std::make_shared<std_msgs::msg::Float64MultiArray>();
    }

  private:

    double wheel_radius_;
    double wheel_distance_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    std::shared_ptr<std_msgs::msg::Float64MultiArray> command_wheel_;

    void cal_inverse_kinematic(float lin_x, float lin_y, float yaw)
    {
        float r = wheel_radius_;
        float R = wheel_distance_;

        float wheel_1 = ( lin_y + R * yaw ) / r;

        float wheel_2 = ( -0.866f * lin_x 
                        -0.5f  * lin_y 
                        + R * yaw ) / r;

        float wheel_3 = (  0.866f * lin_x 
                        -0.5f  * lin_y 
                        + R * yaw ) / r;

        command_wheel_->data = {wheel_1, wheel_2, wheel_3};
    }

    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) 
    {
        cal_inverse_kinematic(msg->linear.x, 
                              msg->linear.y, 
                              msg->angular.z);

        publisher_->publish(*command_wheel_);
    };

    // shared ptr
    

    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Inverse_kinematic>());
  rclcpp::shutdown();
  return 0;
}