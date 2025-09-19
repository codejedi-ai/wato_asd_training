#include "map_memory_core.hpp"
#include <cmath>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger),
    map_width_(200),
    map_height_(200),
    map_resolution_(0.1),
    map_origin_x_(-10.0),
    map_origin_y_(-10.0),
    robot_x_(0.0),
    robot_y_(0.0),
    robot_yaw_(0.0),
    last_update_x_(0.0),
    last_update_y_(0.0),
    distance_threshold_(1.5), // 1.5 meters as specified
    costmap_updated_(false),
    should_update_map_(false)
{
  initializeMap();
}

void MapMemoryCore::initializeMap() {
  global_map_.resize(map_height_, std::vector<signed char>(map_width_, -1)); // Unknown space
  RCLCPP_INFO(logger_, "Initialized global map with size %dx%d", map_width_, map_height_);
}

void MapMemoryCore::updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap) {
  // Store the latest costmap
  latest_costmap_ = *costmap;
  costmap_updated_ = true;
}

void MapMemoryCore::setRobotPose(const nav_msgs::msg::Odometry::SharedPtr odom) {
  if (odom) {
    robot_x_ = odom->pose.pose.position.x;
    robot_y_ = odom->pose.pose.position.y;
    
    // Convert quaternion to yaw
    double x = odom->pose.pose.orientation.x;
    double y = odom->pose.pose.orientation.y;
    double z = odom->pose.pose.orientation.z;
    double w = odom->pose.pose.orientation.w;
    
    robot_yaw_ = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    
    // Check if robot has moved enough distance
    double distance = calculateDistance(robot_x_, robot_y_, last_update_x_, last_update_y_);
    if (distance >= distance_threshold_) {
      last_update_x_ = robot_x_;
      last_update_y_ = robot_y_;
      should_update_map_ = true;
      RCLCPP_DEBUG(logger_, "Robot moved %.2f meters, triggering map update", distance);
    }
  }
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::getMap() const {
  nav_msgs::msg::OccupancyGrid map_msg;
  map_msg.header.stamp = rclcpp::Time();
  map_msg.header.frame_id = "map";
  map_msg.info.resolution = map_resolution_;
  map_msg.info.width = map_width_;
  map_msg.info.height = map_height_;
  map_msg.info.origin.position.x = map_origin_x_;
  map_msg.info.origin.position.y = map_origin_y_;
  map_msg.info.origin.position.z = 0.0;
  map_msg.info.origin.orientation.w = 1.0;
  
  map_msg.data.resize(map_width_ * map_height_);
  for (int y = 0; y < map_height_; ++y) {
    for (int x = 0; x < map_width_; ++x) {
      map_msg.data[y * map_width_ + x] = global_map_[y][x];
    }
  }
  
  return map_msg;
}

bool MapMemoryCore::shouldUpdateMap() const {
  return should_update_map_ && costmap_updated_;
}

void MapMemoryCore::integrateCostmap() {
  if (costmap_updated_) {
    // Fuse the costmap into the global map
    fuseCostmaps(latest_costmap_.data, latest_costmap_.info.width, latest_costmap_.info.height,
                 latest_costmap_.info.origin.position.x, latest_costmap_.info.origin.position.y);
    RCLCPP_DEBUG(logger_, "Integrated costmap into global map");
  }
}

void MapMemoryCore::resetUpdateFlags() {
  should_update_map_ = false;
  costmap_updated_ = false;
}

void MapMemoryCore::fuseCostmaps(const std::vector<signed char>& costmap_data, int costmap_width, int costmap_height, 
                                double costmap_origin_x, double costmap_origin_y) {
  // Transform costmap data to global map coordinates
  for (int cy = 0; cy < costmap_height; ++cy) {
    for (int cx = 0; cx < costmap_width; ++cx) {
      // Convert costmap cell to world coordinates
      double world_x = costmap_origin_x + cx * map_resolution_;
      double world_y = costmap_origin_y + cy * map_resolution_;
      
      // Convert to global map coordinates
      int map_x = worldToMapX(world_x);
      int map_y = worldToMapY(world_y);
      
      if (isValidMapIndex(map_x, map_y)) {
        signed char costmap_value = costmap_data[cy * costmap_width + cx];
        signed char current_value = global_map_[map_y][map_x];
        
        // Map fusion: prioritize new data over old data
        signed char fused_value = mapFusion(current_value, costmap_value);
        global_map_[map_y][map_x] = fused_value;
      }
    }
  }
}

int MapMemoryCore::worldToMapX(double world_x) const {
  return static_cast<int>((world_x - map_origin_x_) / map_resolution_);
}

int MapMemoryCore::worldToMapY(double world_y) const {
  return static_cast<int>((world_y - map_origin_y_) / map_resolution_);
}

bool MapMemoryCore::isValidMapIndex(int x, int y) const {
  return x >= 0 && x < map_width_ && y >= 0 && y < map_height_;
}

signed char MapMemoryCore::mapFusion(signed char current_value, signed char new_value) const {
  // If new value is unknown, retain the previous value
  if (new_value == -1) return current_value;
  
  // If current value is unknown, use the new value
  if (current_value == -1) return new_value;
  
  // If both values are known, prioritize new data over old data
  // This means we overwrite with the new value (new data takes precedence)
  return new_value;
}

double MapMemoryCore::calculateDistance(double x1, double y1, double x2, double y2) const {
  return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

} 
