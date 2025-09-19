#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>
#include <cmath>
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
 
  private:
    // ROS constructs
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    
    // Callback function
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    
    // Costmap processing functions
    void initializeCostmap();
    void convertToGrid(double range, double angle, int& x_grid, int& y_grid);
    void markObstacle(int x_grid, int y_grid);
    void inflateObstacles();
    void publishCostmap();
    
    // Grid parameters (300x300 with center at 150,150)
    static const int grid_width_ = 300;
    static const int grid_height_ = 300;
    static const double grid_resolution_;
    static const double grid_origin_x_;
    static const double grid_origin_y_;
    static const double inflation_radius_;
    
    // Define the grid
    std::vector<std::vector<signed char>> grid_;
};
 
#endif 