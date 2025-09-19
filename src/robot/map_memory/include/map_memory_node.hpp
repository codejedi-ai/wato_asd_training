#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    // Callback functions
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
    void updateMap();
    void publishMap();
    
    // ROS constructs
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr map_timer_;
    
    // Core functionality
    robot::MapMemoryCore map_memory_;
};

#endif 
