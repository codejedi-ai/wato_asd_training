#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <cmath>

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();
    void updateMap(); 

  private:
    // Core functionality
    robot::MapMemoryCore map_memory_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<int8_t> map_;
    nav_msgs::msg::OccupancyGrid map_msg_;
    bool update_ = false, costmap_update_ = false;
    double x_ = 0.0;
    double y_ = 0.0;
    double dis_ = 0.0;
    double new_x_ = 0.0;
    double new_y_ = 0.0;
    geometry_msgs::msg::Quaternion heading_;
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_yaw_ = 0.0;
    nav_msgs::msg::OccupancyGrid global_map_msg_;
    static constexpr int width_ = 300;
    static constexpr int height_ = 300;
    static constexpr double resolution_ = 0.1;
};

#endif 
