#include <iostream>
#include <chrono>
#include <functional>
#include <string>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
using std::placeholders::_1;

class ICP_odometry_laserscan : public rclcpp::Node
{
public:
    ICP_odometry_laserscan(): Node("ICP_estimate_pose"), initialize(false)
    {
        subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ICP_odometry_laserscan::topic_callback, this, _1));
    }

private:

    struct CenterResult
    {
        Eigen::MatrixXf current_dash;
        Eigen::MatrixXf previous_dash;
        Eigen::Vector2f current_bar;
        Eigen::Vector2f previous_bar;
    };

    Eigen::MatrixXf decrypt_polar_coordinate(const sensor_msgs::msg::LaserScan::SharedPtr& laser_coming)
    {
        int N = laser_coming->ranges.size();
        Eigen::MatrixXf answer(N, 2);

        for (int i = 0; i < N; i++)
        {
            float angle = Ang_min + i * Angle_increment;
            float distance = laser_coming->ranges[i];

            if (!std::isfinite(distance))
            {
                answer(i, 0) = 0.0f;
                answer(i, 1) = 0.0f;
                continue;
            }

            answer(i, 0) = distance * std::cos(angle);
            answer(i, 1) = distance * std::sin(angle);
        }

        return answer;
    }

    std::pair<Eigen::MatrixXf, float> find_closest_points(
        const Eigen::MatrixXf& current,
        const Eigen::MatrixXf& previous)
    {
        int N = current.rows();
        Eigen::MatrixXf answer(N, 2);
        float costs = 0.0f;
        int valid_count = 0;

        for (int i = 0; i < N; i++)
        {
            float best_cost = MAXFLOAT;
            Eigen::Vector2f best_point = Eigen::Vector2f::Zero();

            for (int j = 0; j < previous.rows(); j++)
            {
                float dx = current(i, 0) - previous(j, 0);
                float dy = current(i, 1) - previous(j, 1);
                float cost = dx * dx + dy * dy;

                if (cost < best_cost)
                {
                    best_cost = cost;
                    best_point = previous.row(j);
                }
            }

            if (best_cost < threshold)
            {
                answer.row(i) = best_point;
                costs += best_cost;
                valid_count++;
            }
            else
            {
                answer.row(i) = current.row(i); // fallback
            }
        }

        if (valid_count > 0)
            costs /= static_cast<float>(valid_count);
        else
            costs = MAXFLOAT;

        return {answer, costs};
    }

    Eigen::Vector2f centroid(const Eigen::MatrixXf& pts)
    {
        return pts.colwise().mean();
    }

    CenterResult center_the_point(const Eigen::MatrixXf& current,
                                  const Eigen::MatrixXf& previous)
    {
        CenterResult res;

        res.current_bar = centroid(current);
        res.previous_bar = centroid(previous);

        res.current_dash = current.rowwise() - res.current_bar.transpose();
        res.previous_dash = previous.rowwise() - res.previous_bar.transpose();

        return res;
    }

    Eigen::Matrix2f covariance_matrix(const Eigen::MatrixXf& A,
                                      const Eigen::MatrixXf& B)
    {
        Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();

        for (int i = 0; i < A.rows(); i++)
        {
            cov += A.row(i).transpose() * B.row(i);
        }

        return cov;
    }

    void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!initialize)
        {
            Ang_max = msg->angle_max;
            Ang_min = msg->angle_min;
            Angle_increment = msg->angle_increment;
            initialize = true;
        }

        current_laser_message = decrypt_polar_coordinate(msg);

        if (previous_laser_message.rows() > 0)
        {
            Eigen::Matrix2f R_icp = Eigen::Matrix2f::Identity();
            Eigen::Vector2f t_icp = Eigen::Vector2f::Zero();

            for (int i = 0; i < iter; i++)
            {
                auto prev_star =
                    find_closest_points(current_laser_message, previous_laser_message);

                CenterResult center =
                    center_the_point(current_laser_message, prev_star.first);

                Eigen::Matrix2f cov =
                    covariance_matrix(center.current_dash, center.previous_dash);

                Eigen::JacobiSVD<Eigen::Matrix2f> svd(
                    cov, Eigen::ComputeFullU | Eigen::ComputeFullV);

                Eigen::Matrix2f Rot =
                    svd.matrixV() * svd.matrixU().transpose();

                if (Rot.determinant() < 0)
                {
                    Eigen::Matrix2f V = svd.matrixV();
                    V.col(1) *= -1;
                    Rot = V * svd.matrixU().transpose();
                }

                Eigen::Vector2f Tran =
                    center.current_bar - Rot * center.previous_bar;

                R_icp = Rot * R_icp;
                t_icp = Rot * t_icp + Tran;

                if (prev_star.second <= tol)
                    break;
            }

            if (t_icp.norm() < 1.0 && std::isfinite(t_icp.norm()))
            {
                R_total = R_icp * R_total;
                T_total = R_icp * T_total + t_icp;
            }
            else
            {
                std::cout << "Rejected bad ICP step\n";
            }

            if (!std::isfinite(T_total.x()) || !std::isfinite(T_total.y()))
            {
                std::cout << "RESET (NaN detected)\n";
                R_total.setIdentity();
                T_total.setZero();
            }

            std::cout << "Pose: x=" << T_total.x()
                      << " y=" << T_total.y() << std::endl;
        }


        previous_laser_message = current_laser_message;
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;

    Eigen::MatrixXf current_laser_message;
    Eigen::MatrixXf previous_laser_message;

    bool initialize;

    float Ang_max;
    float Ang_min;
    float Angle_increment;

    int iter = 20;
    float tol = 1e-4;
    float threshold = 0.5f;

    Eigen::Matrix2f R_total = Eigen::Matrix2f::Identity();
    Eigen::Vector2f T_total = Eigen::Vector2f::Zero();
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICP_odometry_laserscan>());
    rclcpp::shutdown();
    return 0;
}