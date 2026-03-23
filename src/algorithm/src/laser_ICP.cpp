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
        subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&ICP_odometry_laserscan::topic_callback, this, _1));
    }

private:

    // define CenterResult
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

            answer(i, 0) = distance * std::cos(angle);
            answer(i, 1) = distance * std::sin(angle);
        }

        return answer;
    }


    std::pair<Eigen::MatrixXf, float> find_closest_points(const Eigen::MatrixXf& current_laser_message,const Eigen::MatrixXf& coverge_laser_point)
    {
        int N = current_laser_message.rows();
        float costs = 0.0;
        Eigen::MatrixXf answer(N, 2);

        for (int i = 0; i < N; i++)
        {
            float current_cost = MAXFLOAT;
            Eigen::Vector2f current_point;

            for (int j = 0; j < coverge_laser_point.rows(); j++)
            {
                float dx = current_laser_message(i, 0) - coverge_laser_point(j, 0);
                float dy = current_laser_message(i, 1) - coverge_laser_point(j, 1);
                float cost = dx * dx + dy * dy;

                if (current_cost > cost)
                {
                    current_cost = cost;
                    current_point = coverge_laser_point.row(j);
                }
            }
            costs += current_cost;
            answer.row(i) = current_point;
        }

        costs /= static_cast<float>(N);  

        return {answer, costs};
    }


    Eigen::Vector2f centroid(const Eigen::MatrixXf& point_list)
    {
        return point_list.colwise().mean();
    }

    CenterResult center_the_point(const Eigen::MatrixXf& current, const Eigen::MatrixXf& previous)
    {
        CenterResult answer;

        // finding the centroid point
        answer.current_bar = centroid(current);
        answer.previous_bar = centroid(previous);

        answer.current_dash = current.rowwise() - answer.current_bar.transpose();
        answer.previous_dash = previous.rowwise() - answer.previous_bar.transpose();

        return answer;
    }   

    Eigen::MatrixXf covariance_matrix(const Eigen::MatrixXf& current_dash, const Eigen::MatrixXf& previous_dash)
    {
        Eigen::Matrix2f cov_mat;
        cov_mat.setZero();

        for (int i = 0; i < current_dash.rows(); i++)
        {
            Eigen::Vector2f current_vec = current_dash.row(i);
            Eigen::Vector2f previous_vec = previous_dash.row(i);
            cov_mat += current_vec * previous_vec.transpose();
        }
        
        return cov_mat;
    }

    void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
    {
        if(!initialize)
        {
            Ang_max = msg->angle_max;
            Ang_min = msg->angle_min;
            Angle_increment = msg->angle_increment;
            Ranges = msg->ranges.size();

            initialize = true;
        }
        
        current_laser_message = decrypt_polar_coordinate(msg);

        if (coverge_laser_point.rows() > 0)
        {   

            for (int i = 0; i < iter; i++)
            {
                // we already get the prev_star and the cost
                std::pair<Eigen::MatrixXf, float> prev_star = find_closest_points(current_laser_message, coverge_laser_point);

                // compute the centroid and center the point
                // first -> current_dash, second -> previous_dash
                CenterResult center = center_the_point(current_laser_message ,prev_star.first);

                // making the covariance matrix
                Eigen::MatrixXf cov_matrix = covariance_matrix(center.current_dash, center.previous_dash);

                // finding the SVD(Singular Value Decomposition)
                Eigen::JacobiSVD<Eigen::Matrix2f> svd(cov_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
                
                // finding the rotational
                Eigen::MatrixXf Rot = svd.matrixV() * svd.matrixU().transpose();

                // translation
                Eigen::Vector2f Tran = center.current_bar - Rot*center.previous_bar;   

                coverge_laser_point = (coverge_laser_point * Rot.transpose()).rowwise() + Tran.transpose();

                if (prev_star.second <= tol)
                {
                    std::cout << "Success converge" << std::endl;
                    break;
                }
                
            }
        }   
        else 
        {
            coverge_laser_point = current_laser_message;
        }

        coverge_laser_point = current_laser_message;
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;

    Eigen::MatrixXf current_laser_message;
    Eigen::MatrixXf coverge_laser_point;

    // covert each iteration into Rotational and translation matrix
    Eigen::MatrixXf Rot;
    Eigen::MatrixXf Tran;

    // single init flag
    bool initialize;

    // laser properties
    float Ang_max;
    float Ang_min;
    float Angle_increment;
    int Ranges;

    // set the iteration and tolerance
    int iter = 20;
    float tol = 1e-4;
};