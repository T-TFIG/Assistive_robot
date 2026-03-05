#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <termios.h>
#include <unistd.h>
#include <stdio.h>

#include <teleop_twist_parameters.hpp>

class KeyboardTeleop : public rclcpp::Node
{
public:
    KeyboardTeleop() : Node("keyboard_teleop")
    {
        param_listener_ =
            std::make_shared<teleop_keyboard::ParamListener>(this);

        params_ = param_listener_->get_params();

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            params_.topic_name, 10);


        run();
    }

private:

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    std::shared_ptr<teleop_keyboard::ParamListener> param_listener_;
    teleop_keyboard::Params params_;

    int getch()
    {
        struct termios oldt, newt;
        int ch;

        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);

        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        ch = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

        return ch;
    }

    void run()
    {
        geometry_msgs::msg::Twist msg;

        while (rclcpp::ok())
        {
            char c = getch();

            msg.linear.x = 0.0;
            msg.linear.y = 0.0;
            msg.angular.z = 0.0;

            switch(c)
            {
                case 'q':
                    msg.linear.x = -0.5;
                    msg.linear.y = 0.5;
                    break;

                case 'w':
                    msg.linear.y = 0.5;
                    break;

                case 'e':
                    msg.linear.x = 0.5;
                    msg.linear.y = 0.5;
                    break;

                case 'a':
                    msg.linear.x = -0.5;
                    break;

                case 's':
                    break;

                case 'd':
                    msg.linear.x = 0.5;
                    break;

                case 'z':
                    msg.linear.x = -0.5;
                    msg.linear.y = -0.5;
                    break;

                case 'x':
                    msg.linear.y = -0.5;
                    break;

                case 'c':
                    msg.linear.x = 0.5;
                    msg.linear.y = -0.5;
                    break;

                case 'r':
                    msg.angular.z = 0.2;
                    break;

                case 't':
                    msg.angular.z = -0.2;
                    break;

                case 'b':
                    rclcpp::shutdown();
                    return;
            }

            publisher_->publish(msg);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardTeleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}