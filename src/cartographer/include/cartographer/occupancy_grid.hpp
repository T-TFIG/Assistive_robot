#ifndef CARTOGRAPHER__OCCUPANCY_GRID_HPP_
#define CARTOGRAPHER__OCCUPANCY_GRID_HPP_

#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <numbers>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>


namespace cartographer
{

class OccupancyGrid
{
public:
    // ── constructor ───────────────────────────────────────────────────────────
    OccupancyGrid(int width, int height, float resolution,
                  float origin_x, float origin_y);

    // ── scan preprocessing ────────────────────────────────────────────────────
    // converts raw LaserScan (polar) into clean Cartesian points in robot frame
    // returns: vector of (x,y) points, invalid rays already filtered out
    static std::vector<Eigen::Vector2f> laser_to_cartesian(
        const sensor_msgs::msg::LaserScan & scan);

    // ── coordinate conversion ─────────────────────────────────────────────────
    void world_to_cell(float wx, float wy, int & cx, int & cy) const;
    void cell_to_world(int cx, int cy, float & wx, float & wy) const;
    bool in_bounds(int cx, int cy) const;

    // ── cell read ─────────────────────────────────────────────────────────────
    float get_log_odds(int cx, int cy) const;
    float get_probability(int cx, int cy) const;   // converts L → P(occ)

    // ── log-odds update ───────────────────────────────────────────────────────
    void update_hit(int cx, int cy);
    void update_free(int cx, int cy);

    // ── main update entry point ───────────────────────────────────────────────
    // takes clean Cartesian points + robot pose, runs Bresenham, updates grid
    void insert_scan(const std::vector<Eigen::Vector2f> & points,
                     float robot_x, float robot_y, float robot_theta);

    // ── scan matcher support  (Stage 2 — Gauss-Newton) ───────────────────────
    float interpolate(float wx, float wy) const;
    Eigen::Vector2f gradient(float wx, float wy) const;

    // ── RTCSM pyramid support  (Stage 1 — branch-and-bound) ──────────────────
    OccupancyGrid downsample(int factor) const;

    // ── ROS output ────────────────────────────────────────────────────────────
    nav_msgs::msg::OccupancyGrid to_ros_msg(const std::string & frame_id,
                                             const rclcpp::Time & stamp) const;

    int   width()      const { return width_; }
    int   height()     const { return height_; }
    float resolution() const { return resolution_; }
    float origin_x()   const { return origin_x_; }
    float origin_y()   const { return origin_y_; }

private:
    std::vector<float> log_odds_;   
    int   width_;
    int   height_;
    float resolution_;              // metres per cell
    float origin_x_;               // world X coordinate of cell (0,0)
    float origin_y_;               // world Y coordinate of cell (0,0)

    // sensor model constants — tune L_OCC and L_FREE for your environment
    static constexpr float L_OCC  =  0.85f;
    static constexpr float L_FREE = -0.40f;
    static constexpr float L_MIN  = -3.5f;
    static constexpr float L_MAX  =  3.5f;

    // internal helper called by insert_scan
    void bresenham(int x0, int y0, int x1, int y1);

};

}  // namespace cartographer

#endif  // CARTOGRAPHER__OCCUPANCY_GRID_HPP_
