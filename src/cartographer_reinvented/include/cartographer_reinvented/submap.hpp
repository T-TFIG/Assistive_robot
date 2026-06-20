#ifndef CARTOGRAPHER_REINVENTED__SUBMAP_HPP_
#define CARTOGRAPHER_REINVENTED__SUBMAP_HPP_

#include "cartographer_reinvented/occupancy_grid.hpp"


namespace cartographer_reinvented
{

class Submap
{
public:
    Submap(int width, int height, float resolution,
           float origin_x, float origin_y,
           int max_scans = 50);

    // insert one scan into this submap (calls grid_.insert_scan internally)
    void insert_scan(const std::vector<Eigen::Vector2f> & points,
                     float robot_x, float robot_y, float robot_theta);

    bool is_full() const;   // true when scan_count_ >= max_scans_
    void freeze();          // lock the submap — no more updates

    // read-only access to the grid (for publishing and scan matching)
    const OccupancyGrid & grid() const { return grid_; }
    bool frozen() const { return frozen_; }
    int  scan_count() const { return scan_count_; }

private:
    OccupancyGrid grid_;
    int  scan_count_;
    int  max_scans_;
    bool frozen_;
};

}  // namespace cartographer_reinvented

#endif  // CARTOGRAPHER_REINVENTED__SUBMAP_HPP_
