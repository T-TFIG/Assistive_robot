#include "cartographer_reinvented/submap_manager.hpp"


namespace cartographer_reinvented
{
    SubmapManager::SubmapManager(int width, int height, float resolution, int max_scans_per_submap)
    : width_(width), height_(height), resolution_(resolution), max_scans_per_submap_(max_scans_per_submap){}

    void SubmapManager::add_scan(const std::vector<Eigen::vector2f> & points, float robot_x, float robot_y, float robot_theta)
    {
        
    }


}