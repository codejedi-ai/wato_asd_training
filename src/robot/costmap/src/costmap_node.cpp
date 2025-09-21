#include "costmap_node.hpp"
#include "costmap_core.hpp"

CostmapNode::CostmapNode() : Node("costmap"), core_(this->get_logger()) {
  // Create subscribers and publishers
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  
  RCLCPP_INFO(this->get_logger(), "Costmap Node started");
}
 

void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // Process laser scan using core
    core_.processLaserScan(scan);
    
    // Get costmap from core and publish
    nav_msgs::msg::OccupancyGrid costmap_msg = core_.getCostmap();
    costmap_msg.header.stamp = this->now();
    costmap_pub_->publish(costmap_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}