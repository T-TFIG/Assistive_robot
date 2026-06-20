#include "cartographer_reinvented/submap.hpp"

namespace cartographer_reinvented
{
    Submap::Submap(int width, int height, float resolution, float origin_x, float origin_y, int max_scans)
    : grid_(width, height, resolution, origin_x, origin_y),
    scan_count_(0),
    max_scans_(max_scans),
    frozen_(false)
    {
    }


    void Submap::insert_scan(const std::vector<Eigen::Vector2f> & points,
                             float robot_x, float robot_y, float robot_theta)
    {   
        if (frozen_)
        {
            return;
        }

        grid_.insert_scan(points, robot_x, robot_y, robot_theta);

        scan_count_++;

        if (is_full())
        {
            freeze();
        }
        
    }

    bool Submap::is_full() const
    {
        return scan_count_ >= max_scans_;
    }

    void Submap::freeze()
    {
        frozen_ = true;
    }

    
}