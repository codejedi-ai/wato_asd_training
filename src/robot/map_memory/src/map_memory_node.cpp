#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <cmath>
#include <memory>

class MappingNode : public rclcpp::Node {
  public:
      MappingNode() : Node("mapping_node"), last_x(0.0), last_y(0.0), distance_threshold(1.5) {
          // Initialize subscribers
          costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
              "/costmap", 10, std::bind(&MappingNode::costmapCallback, this, std::placeholders::_1));
          odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
              "/odom/filtered", 10, std::bind(&MappingNode::odomCallback, this, std::placeholders::_1));
   
          // Initialize publisher
          map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
   
          // Initialize timer
          timer_ = this->create_wall_timer(
              std::chrono::seconds(1), std::bind(&MappingNode::updateMap, this));
          
          // Initialize global map
          initializeGlobalMap();
      }
   
  private:
      // Subscribers and Publisher
      rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
      rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
      rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
      rclcpp::TimerBase::SharedPtr timer_;
   
      // Global map and robot position
      nav_msgs::msg::OccupancyGrid global_map_;
      nav_msgs::msg::OccupancyGrid latest_costmap_;
      double last_x, last_y;
      const double distance_threshold;
      bool costmap_updated_ = false;
      bool should_update_map_ = false;
      bool map_initialized_ = false;
   
      // Callback for costmap updates
      void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          // Store the latest costmap
          latest_costmap_ = *msg;
          costmap_updated_ = true;
      }
   
      // Callback for odometry updates
      void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
          double x = msg->pose.pose.position.x;
          double y = msg->pose.pose.position.y;
   
          // Compute distance traveled
          double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));
          if (distance >= distance_threshold) {
              last_x = x;
              last_y = y;
              should_update_map_ = true;
          }
      }
   
      // Timer-based map update
      void updateMap() {
          if (should_update_map_ && costmap_updated_) {
              integrateCostmap();
              map_pub_->publish(global_map_);
              should_update_map_ = false;
          }
      }
   
      // Initialize the global map
      void initializeGlobalMap() {
          global_map_.header.frame_id = "map";
          global_map_.info.resolution = 0.1; // 10cm resolution (matches costmap)
          global_map_.info.width = 300; // 30m x 30m map (matches costmap)
          global_map_.info.height = 300;
          global_map_.info.origin.position.x = 150.0; // Center at (150, 150) like costmap
          global_map_.info.origin.position.y = 150.0;
          global_map_.info.origin.position.z = 0.0;
          global_map_.info.origin.orientation.w = 0.0; // Match costmap orientation
          
          // Initialize all cells as unknown (-1)
          global_map_.data.resize(global_map_.info.width * global_map_.info.height, -1);
          map_initialized_ = true;
      }

      // Convert cost values to occupancy values
      signed char costToOccupancy(signed char cost_value) {
          if (cost_value == 0) {
              return 0; // Free space
          } else if (cost_value == 100) {
              return 100; // Occupied
          } else if (cost_value > 0) {
              return 100; // Treat any cost as occupied for simplicity
          } else {
              return -1; // Unknown
          }
      }

      // Integrate the latest costmap into the global map
      void integrateCostmap() {
          if (!map_initialized_ || latest_costmap_.data.empty()) {
              return;
          }

          // Both costmap and global map have the same format: 300x300 with center at (150, 150)
          // Since lidar scans 360 degrees, the costmap is already in global coordinates
          // We can directly merge the costmap into the global map
          
          // Calculate robot position offset in global map coordinates
          int robot_offset_x = static_cast<int>(last_x / global_map_.info.resolution);
          int robot_offset_y = static_cast<int>(last_y / global_map_.info.resolution);
          
          // Merge the costmap into the global map
          for (unsigned int i = 0; i < latest_costmap_.info.width; i++) {
              for (unsigned int j = 0; j < latest_costmap_.info.height; j++) {
                  // Calculate global map indices (center the costmap at robot position)
                  int global_i = i + robot_offset_x;
                  int global_j = j + robot_offset_y;
                  
                  // Check bounds
                  if (global_i >= 0 && global_i < static_cast<int>(global_map_.info.width) && 
                      global_j >= 0 && global_j < static_cast<int>(global_map_.info.height)) {
                      
                      int costmap_idx = j * latest_costmap_.info.width + i;
                      int global_idx = global_j * global_map_.info.width + global_i;
                      
                      // Convert cost value to occupancy value and merge
                      signed char cost_value = latest_costmap_.data[costmap_idx];
                      signed char occupancy_value = costToOccupancy(cost_value);
                      
                      // Only update if the new cell has a known value (not unknown)
                      if (occupancy_value != -1) {
                          global_map_.data[global_idx] = occupancy_value;
                      }
                  }
              }
          }
          
          // Update the global map header
          global_map_.header.stamp = this->now();
      }
  };

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MappingNode>());
    rclcpp::shutdown();
    return 0;
}