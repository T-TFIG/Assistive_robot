#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class SimpleTwistMux : public rclcpp::Node {
public:
    SimpleTwistMux() : Node("simple_twist_mux") {
        // 1. Initialize last_msg_time_ with the current clock source right away
        last_msg_time_ = this->now();

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/omni_base_controller/cmd_vel", 10);

        nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                handle_msg(msg, false); 
            });

        manual_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/omni_base_controller/keyboard", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                handle_msg(msg, true); 
            });

        timer_ = this->create_wall_timer(100ms, std::bind(&SimpleTwistMux::failsafe_callback, this));
    }

private:
    void handle_msg(const geometry_msgs::msg::Twist::SharedPtr msg, bool is_manual) {
        last_msg_time_ = this->now();
        
        if (is_manual) {
            manual_active_ = true;
            publisher_->publish(*msg);
        } else if (!manual_active_) {
            publisher_->publish(*msg);
        }
    }

    void failsafe_callback() {
        // 2. Wrap in a try-catch or check if time is zero to be extra safe
        try {
            auto age = this->now() - last_msg_time_;
            if (age > 500ms) {
                manual_active_ = false;
            }
        } catch (const std::runtime_error & e) {
            // If the clock hasn't started yet (common in Gazebo startup)
            return;
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_msg_time_;
    bool manual_active_ = false;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleTwistMux>());
    rclcpp::shutdown();
    return 0;
}