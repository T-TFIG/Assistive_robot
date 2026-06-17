#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

using std::placeholders::_1;

class GroundTruthPublisher : public rclcpp::Node
{
public:
    GroundTruthPublisher()
    : Node("ground_truth_publisher")
    {
        pose_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/world/default/dynamic_pose/info", 10,
            std::bind(&GroundTruthPublisher::pose_callback, this, _1));

        gt_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/ground_truth", 10);
    }

private:
    void pose_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        for (const auto & transform : msg->transforms) {
            if (transform.child_frame_id == "mobile_robot") {
                nav_msgs::msg::Odometry odom_msg;
                odom_msg.header.stamp    = transform.header.stamp;
                odom_msg.header.frame_id = "odom";
                odom_msg.child_frame_id  = "base_link";

                odom_msg.pose.pose.position.x = transform.transform.translation.x;
                odom_msg.pose.pose.position.y = transform.transform.translation.y;
                odom_msg.pose.pose.position.z = transform.transform.translation.z;

                odom_msg.pose.pose.orientation.x = transform.transform.rotation.x;
                odom_msg.pose.pose.orientation.y = transform.transform.rotation.y;
                odom_msg.pose.pose.orientation.z = transform.transform.rotation.z;
                odom_msg.pose.pose.orientation.w = transform.transform.rotation.w;

                gt_pub_->publish(odom_msg);
                return;
            }
        }
    }

    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gt_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundTruthPublisher>());
    rclcpp::shutdown();
    return 0;
}
