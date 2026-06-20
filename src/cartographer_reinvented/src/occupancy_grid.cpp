#include "cartographer_reinvented/occupancy_grid.hpp"

namespace cartographer_reinvented
{
    OccupancyGrid::OccupancyGrid(int width, int height, float resolution, float origin_x, float origin_y): 
    width_(width), height_(height), resolution_(resolution), origin_x_(origin_x), origin_y_(origin_y) {}


    static std::vector<Eigen::Vector2f> laser_to_cartesian(sensor_msgs::msg::LaserScan & scan)
    {
        double angle_min = scan.angle_min;
        double angle_max = scan.angle_max;
        double angle_increment = scan.angle_increment;
        double range_min = scan.range_min;
        double range_max = scan.range_max;

        int number_of_point = scan.ranges.size();

        std::vector<Eigen::Vector2f> cartesian_space;

        for (int i = 0; i < number_of_point; i++)
        {
            double curr_scan_ranges = scan.ranges[i];

            if (curr_scan_ranges >= range_min && 
                curr_scan_ranges <= range_max &&
                !std::isnan(curr_scan_ranges) &&
                !std::isinf(curr_scan_ranges))
            {
                double curr_angle = angle_min + (i * angle_increment);
                Eigen::Vector2f cur_cartesian(curr_scan_ranges * std::cos(curr_angle), curr_scan_ranges * std::sin(curr_angle));
                cartesian_space.push_back(cur_cartesian);
            }
        
        }   

        return cartesian_space;
        
    }


    void OccupancyGrid::world_to_cell(float wx, float wy, int & cx, int & cy) const
    {
        cx = static_cast<int>((wx - origin_x()) / resolution());
        cy = static_cast<int>((wy - origin_y()) / resolution());
    }


    void OccupancyGrid::cell_to_world(int cx, int cy, float & wx, float & wy) const
    {
        wx = (static_cast<float>(cx) * resolution()) + origin_x() + (resolution()/2);
        wy = (static_cast<float>(cy) * resolution()) + origin_y() + (resolution()/2);
    }

    bool OccupancyGrid::in_bounds(int cx, int cy) const
    {
        if (cx < 0 || cx >= width() || cy < 0 || cy >= height())
        {
            return false; 
        }
        
        return true; 
    }

    
    float OccupancyGrid::get_log_odds(int cx, int cy) const
    {
        if(!in_bounds(cx, cy))
        {
            return 0.0f;
        }

        int index = (cy * width_) + cx;

        return log_odds_[index];
    }

    float OccupancyGrid::get_probability(int cx, int cy) const
    {
        float L = get_log_odds(cx, cy);

        return 1.0f/(1.0f + std::exp(-L));
    }

    void OccupancyGrid::update_hit(int cx, int cy)
    {
        if (!in_bounds(cx, cy))
        {
            return;
        }

        float L = get_log_odds(cx, cy);
        float L_new = std::min(L + L_OCC, L_MAX);

        int index = (cy * width_) + cx;
        log_odds_[index] = L_new;
    }

    void OccupancyGrid::update_free(int cx, int cy)
    {
        if (!in_bounds(cx, cy))
        {
            return;
        }

        float L = get_log_odds(cx, cy);
        float L_new = std::max(L + L_FREE, L_MIN);

        int index = (cy * width_) + cx;
        log_odds_[index] = L_new;

    }

    void OccupancyGrid::insert_scan(const std::vector<Eigen::Vector2f> & points, float robot_x, float robot_y, float robot_theta)
    {
        int robot_cx = 0, robot_cy = 0;
        world_to_cell(robot_x, robot_y, robot_cx, robot_cy);

        float cos_theta = std::cos(robot_theta);
        float sin_theta = std::sin(robot_theta);

        for (const auto & p_local : points)
        {
            float wx = (p_local.x() * cos_theta) - (p_local.y() * sin_theta) + robot_x;
            float wy = (p_local.x() * sin_theta) + (p_local.y() * cos_theta) + robot_y;

            int hit_cx = 0, hit_cy = 0;
            world_to_cell(wx, wy, hit_cx, hit_cy);

            bresenham(robot_cx, robot_cy, hit_cx, hit_cy);

            update_hit(hit_cx, hit_cy);
        }
        
    }

    void OccupancyGrid::bresenham(int x0, int y0, int x1, int y1)
    {
        int dx = std::abs(x1 - x0);
        int dy = std::abs(y1 - y0);

        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;

        int err = dx - dy;

        while (true)
        {
            if (x0 == x1 && y0 == y1)
            {
                break;
            }

            update_free(x0, y0);

            int e2 = 2 * err;

            if (e2 > -dy)
            {
                err -= dy;
                x0 += sx;
            }

            if (e2 < dx)
            {
                err += dx;
                y0 += sy;
            }   
        }
    }

    nav_msgs::msg::OccupancyGrid OccupancyGrid::to_ros_msg(const std::string & frame_id, const rclcpp::Time & stamp) const
    {
        nav_msgs::msg::OccupancyGrid msg;
        geometry_msgs::msg::Pose msg_pose;
        
        // origin pose
        msg_pose.position.x = origin_x();
        msg_pose.position.y = origin_y();
        msg_pose.orientation.w = 1.0;

        // header
        msg.header.frame_id = frame_id;
        msg.header.stamp = stamp;

        // information
        msg.info.resolution = resolution();
        msg.info.width = width();
        msg.info.height = height();
        msg.info.origin = msg_pose;

        // data
        msg.data.resize(width() * height());

        for (int cy = 0; cy < height(); ++cy)
        {
            for (int cx = 0; cx < width(); ++cx)
            {
                int index = (cy * width()) + cx;

                float log_odd_val = get_log_odds(cx, cy);

                if (log_odd_val == 0.0f)
                {
                    msg.data[index] = -1;
                }
                else
                {
                    float prob = get_probability(cx, cy);

                    msg.data[index] = static_cast<int8_t>(std::round(prob * 100.0f));
                }
                
            }
            
        }

        return msg;
        
    }

    float OccupancyGrid::interpolate(float wx, float wy) const
    {
        float x_grid = (wx - origin_x())/resolution();
        float y_grid = (wy - origin_y())/resolution();

        int x0 = static_cast<int>(std::floor(x_grid));
        int y0 = static_cast<int>(std::floor(y_grid));
        int x1 = x0 + 1;
        int y1 = y0 + 1;

        if (!in_bounds(x0, y0) || !in_bounds(x0, y1) 
        || !in_bounds(x1, y0) || !in_bounds(x1, y1))
        {
            return 0.5f;
        }

        float delta_x = x_grid - static_cast<float>(x0);
        float delta_y = y_grid - static_cast<float>(y0);

        float p00 = get_probability(x0, y0);
        float p01 = get_probability(x0, y1);
        float p10 = get_probability(x1, y0);
        float p11 = get_probability(x1, y1);

        float interpolated_prob = (1.0f - delta_x) * (1.0f - delta_y) * p00 +
                                  delta_x * (1.0f - delta_y) * p10 + 
                                  (1.0f - delta_x) * delta_y * p01 + 
                                  delta_x * delta_y * p11;

        return interpolated_prob;
    }


    Eigen::Vector2f OccupancyGrid::gradient(float wx, float wy) const
    {
        float step = resolution_;

        float active_x_plus = interpolate(wx + step, wy);
        float active_x_minus = interpolate(wx - step, wy);
        float active_y_plus = interpolate(wx, wy + step);
        float active_y_minus = interpolate(wx, wy - step);

        float grad_x = (active_x_plus - active_x_minus) / (2.0f * step);
        float grad_y = (active_y_plus - active_y_minus) / (2.0f * step);

        return Eigen::Vector2f(grad_x, grad_y);
    }

    OccupancyGrid OccupancyGrid::downsample(int factor) const
    {
        int new_width = width_ / factor;
        int new_height = height_ / factor;
        int new_resolution = resolution_ / factor;

        OccupancyGrid down_sampling(new_width, new_height, new_resolution, origin_x_, origin_y_);

        for (int new_cy = 0; new_cy < new_height; ++new_cy)
        {
            for (int new_cx = 0; new_cx < new_width; ++new_cx)
            {
                float max_log_odds = L_MIN;

                for (int dy = 0; dy < factor; ++dy)
                {
                    for (int dx = 0; dx < factor; ++dx)
                    {
                        int old_cx = (new_cx * factor) + dx;
                        int old_cy = (new_cy * factor) + dy;

                        if (in_bounds(old_cx, old_cy))
                        {
                            max_log_odds = std::max(max_log_odds, get_log_odds(old_cx, old_cy));
                        }
                    }
                    
                }

                int new_index = (new_cy * new_width) + new_cx;
                down_sampling.log_odds_[new_index] = max_log_odds;
             
            }
            
        }

        return down_sampling;
    }




    // ดูเรื่อง bilinear interpolate ด้วย 
    // กับ bresenham
    // แล้วก็ gredient ว่ามาจากอะไร












}