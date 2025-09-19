#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <vector>
#include <memory>

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);
    
    // Public methods for the node to call
    void updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap);
    nav_msgs::msg::OccupancyGrid getMap() const;
    void setRobotPose(const nav_msgs::msg::Odometry::SharedPtr odom);
    bool shouldUpdateMap() const;
    void integrateCostmap();
    void resetUpdateFlags();

  private:
    rclcpp::Logger logger_;
    
    // Map parameters
    int map_width_;
    int map_height_;
    double map_resolution_;
    double map_origin_x_;
    double map_origin_y_;
    
    // Global map storage
    std::vector<std::vector<signed char>> global_map_;
    
    // Robot pose tracking
    double robot_x_;
    double robot_y_;
    double robot_yaw_;
    double last_update_x_;
    double last_update_y_;
    double distance_threshold_;
    
    // Latest costmap storage
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool costmap_updated_;
    bool should_update_map_;
    
    // Helper methods
    void initializeMap();
    void fuseCostmaps(const std::vector<signed char>& costmap_data, int costmap_width, int costmap_height, 
                     double costmap_origin_x, double costmap_origin_y);
    int worldToMapX(double world_x) const;
    int worldToMapY(double world_y) const;
    bool isValidMapIndex(int x, int y) const;
    signed char mapFusion(signed char current_value, signed char new_value) const;
    double calculateDistance(double x1, double y1, double x2, double y2) const;
};

}  

#endif  
