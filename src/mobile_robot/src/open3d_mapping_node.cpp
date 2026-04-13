#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cv_bridge/cv_bridge.h>
#include <open3d/Open3D.h>
#include <Eigen/Geometry>

class Open3DMapper : public rclcpp::Node
{
public:
    Open3DMapper() : Node("open3d_mapping_node")
    {
        //Setup the TSDF column the 3d map container which voxel size is 0.01m and sdf truncation = 0.04m
        volume_ = std::make_shared<open3d::pipelines::integration::ScalableTSDFVolume>(
            0.01, 0.04, open3d::pipelines::integration::TSDFVolumeColorType::Gray8);

        // setting the intrinsic_ we take it from the realsense D455
        /*
            the intrinsic_ is define the internal optical and geometric characteristics of a camera,
            mapping 3D camera-space coordinates to 2D pixel coordinates 

            we can find those parameter from the checkerboard 
        */
        intrinsic_ = open3d::camera::PinholeCameraIntrinsic(
            640, 480, 381.3, 381.3, 320.0, 240.0);

        // subscriber part
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&Open3DMapper::odom_callback, this, std::placeholders::_1));

        // this is get the 2d image with the Planar Depth meaning that it given you the z-axis depth inside this image
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/base_cam/camera/depth/image_raw", 10, std::bind(&Open3DMapper::depth_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Open3D Mapping Node Started.");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Convert Odom Pose to 4x4 Eigen Matrix
        Eigen::Quaterniond q(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z);
        
        current_pose_ = Eigen::Matrix4d::Identity();
        current_pose_.block<3, 3>(0, 0) = q.toRotationMatrix();
        current_pose_(0, 3) = msg->pose.pose.position.x;
        current_pose_(1, 3) = msg->pose.pose.position.y;
        current_pose_(2, 3) = msg->pose.pose.position.z;
    }

    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            // 1. Convert ROS Image to OpenCV
            // so basically we trying to conver that message into OpenCV Mat and we convert those value into 16-bit unsigned 
            // that mean each of the value inside the matrix can be a depth value from 0 - 65,535 mm 
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            

            // 2. Convert OpenCV to Open3D Image
            // we tells open3d to clear some memory space.The 1, 2 means 1 channel(depth) and 2 bytes 
            // per pixel (which is 16 bits).
            auto o3d_depth = std::make_shared<open3d::geometry::Image>();
            o3d_depth->Prepare(cv_ptr->image.cols, cv_ptr->image.rows, 1, 2);

            // we copy the memoty from the OpenCV and dump it into the Open3D memory buffer
            std::memcpy(o3d_depth->data_.data(), cv_ptr->image.data, o3d_depth->data_.size());


            // 3. Create RGBD Image (Depth only in this case)
            // we didn't use any of the color right now so we just pass it first 
            // the 1000 is to convert from the mm to m by divide it with 1000
            // 3.0 is told the Open3D to ignore the value that is far aways than 3 meters ignore all of those
            auto rgbd = open3d::geometry::RGBDImage::CreateFromColorAndDepth(
                open3d::geometry::Image(), *o3d_depth, 1000.0, 3.0, false);

            // 4. Integrate into Map using the latest Odometry pose
            // Note: We use the inverse because Open3D wants World-to-Camera
            volume_->Integrate(*rgbd, intrinsic_, current_pose_.inverse());

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Integrating frames...");

        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Mapping error: %s", e.what());
        }
    }

    // Member variables
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    
    std::shared_ptr<open3d::pipelines::integration::ScalableTSDFVolume> volume_;
    open3d::camera::PinholeCameraIntrinsic intrinsic_;
    Eigen::Matrix4d current_pose_ = Eigen::Matrix4d::Identity();
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Open3DMapper>());
    rclcpp::shutdown();
    return 0;
}