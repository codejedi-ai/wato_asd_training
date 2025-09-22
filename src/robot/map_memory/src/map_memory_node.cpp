#include "map_memory_node.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MapMemoryNode::timerCallback, this));
  map_ = std::vector<int8_t>(300 * 300, -1);
  global_map_msg_ = nav_msgs::msg::OccupancyGrid();
  global_map_msg_.header.stamp = this->now();
  global_map_msg_.header.frame_id = "sim_world";
  global_map_msg_.info.resolution = resolution_;
  global_map_msg_.info.width = width_;
  global_map_msg_.info.height = height_;
  global_map_msg_.info.origin.position.x = -width_ * resolution_ / 2.0;
  global_map_msg_.info.origin.position.y = -height_ * resolution_ / 2.0;
  updateMap();
  map_pub_->publish(global_map_msg_);
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  map_msg_ = *msg;
  costmap_update_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  new_x_ = msg->pose.pose.position.x;
  new_y_ = msg->pose.pose.position.y;
  robot_x_ = new_x_;
  robot_y_ = new_y_;
  dis_ += std::sqrt((new_x_ - x_) * (new_x_ - x_) + (new_y_ - y_) * (new_y_ - y_));
  
  heading_ = msg->pose.pose.orientation;
  double roll, pitch, yaw;
  tf2::Quaternion q(heading_.x, heading_.y, heading_.z, heading_.w);
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  robot_yaw_ = yaw;
  
  if (dis_ > 1.5) {
    update_ = true;
    x_ = new_x_;
    y_ = new_y_;
    dis_ = 0.0;
  }
}

void MapMemoryNode::updateMap() {
  if (std::isnan(robot_x_) || std::isnan(robot_y_)) {
    return;
  }
  for (auto y = 0; y < static_cast<int>(map_msg_.info.height); y++) {
    for (auto x = 0; x < static_cast<int>(map_msg_.info.width); x++) {
      int index = y * map_msg_.info.width + x;
      if (index < 0 || index >= static_cast<int>(map_msg_.data.size())){
        continue;
      }
      double local_x = (x - static_cast<int>(map_msg_.info.width) / 2) * map_msg_.info.resolution;
      double local_y = (y - static_cast<int>(map_msg_.info.height) / 2) * map_msg_.info.resolution;
      
      double global_x = robot_x_ + local_x * std::cos(robot_yaw_) - local_y * std::sin(robot_yaw_);
      double global_y = robot_y_ + local_x * std::sin(robot_yaw_) + local_y * std::cos(robot_yaw_);

      int idx_x = static_cast<int>(std::round(global_x / resolution_ + width_ / 2));
      int idx_y = static_cast<int>(std::round(global_y / resolution_ + height_ / 2));
      if (idx_x >= 0 && idx_x < width_ && idx_y >= 0 && idx_y < height_) {
        map_[idx_y * width_ + idx_x] = std::max(map_[idx_y * width_ + idx_x], static_cast<int8_t>(map_msg_.data[index]));
      }
    }
  }
  global_map_msg_.header.stamp = this->now();
  global_map_msg_.data = map_;
  map_pub_->publish(global_map_msg_);
  update_ = false;
  costmap_update_ = false;
}

void MapMemoryNode::timerCallback() {
  if (!update_ || !costmap_update_) {
    return;
  }
  updateMap();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}