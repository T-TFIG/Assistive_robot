#ifndef CARTOGRAPHER__SUBMAP_MANAGER_HPP_
#define CARTOGRAPHER__SUBMAP_MANAGER_HPP_

#include "cartographer/submap.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

namespace cartographer
{

class SubmapManager
{
public:
    SubmapManager(int width, int height, float resolution,
                  int max_scans_per_submap = 50);

    // called on every scan: inserts into the active submap,
    // creates a new one automatically when the active is full
    void add_scan(const std::vector<Eigen::Vector2f> & points,
                  float robot_x, float robot_y, float robot_theta);

    // merges all submap grids into one global map for /map publishing
    nav_msgs::msg::OccupancyGrid get_global_map(const std::string & frame_id,
                                                  const rclcpp::Time & stamp) const;

    // for loop closure: access frozen submaps
    const std::vector<Submap> & submaps() const { return submaps_; }

private:
    std::vector<Submap> submaps_;
    int   width_;
    int   height_;
    float resolution_;
    int   max_scans_per_submap_;

    void create_new_submap(float origin_x, float origin_y);
};

}  // namespace cartographer

#endif  // CARTOGRAPHER__SUBMAP_MANAGER_HPP_
