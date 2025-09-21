#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>
#include <cmath>

namespace robot
{

class CostmapCore {
  public:
    explicit CostmapCore(const rclcpp::Logger& logger);
    
    // Public methods for the node to call
    void processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    nav_msgs::msg::OccupancyGrid getCostmap() const;
    void resetCostmap();

  private:
    rclcpp::Logger logger_;
    
    // Grid parameters
    static const int grid_width_ = 300;
    static const int grid_height_ = 300;
    static const double grid_resolution_;
    static const double grid_origin_x_;
    static const double grid_origin_y_;
    static const double inflation_radius_;
    
    // Grid storage
    std::vector<std::vector<signed char>> grid_;
    
    // Processing methods
    void initializeCostmap();
    void convertToGrid(double range, double angle, int& x_grid, int& y_grid);
    void markObstacle(int x_grid, int y_grid);
    void inflateObstacles();
    void publishCostmap();
    
    // Helper methods
    bool isValidGridIndex(int x, int y) const;
    double calculateDistance(int x1, int y1, int x2, int y2) const;
};

}  

#endif  