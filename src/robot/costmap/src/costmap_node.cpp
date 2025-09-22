#include <chrono>
#include <memory>
#include <string>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", 10);
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::laserScanCallback, this, std::placeholders::_1));
}

void CostmapNode::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  static constexpr int width = 300;
  static constexpr int height = 300;
  static constexpr double resolution = 0.1;
  static constexpr int default_value = 0;
  static constexpr int max_cost = 100;
  static constexpr double inflation_radius = 0.5;

  std::vector<int8_t> costmap(width * height, default_value);

  for (size_t i = 0; i < msg->ranges.size(); i++) {
    double range = msg->ranges[i];
    double angle = msg->angle_min + i * msg->angle_increment;

    //check if the point is in the range of the laser scan
    if (range < msg->range_min || range > msg->range_max || std::isnan(range)) {
      continue;
    }
      
    double x = range * cos(angle);
    double y = range * sin(angle);
    int x_grid = x / resolution+width/2;
    int y_grid = y / resolution+height/2;

    //check if the point is in the costmap
    if (x_grid < 0 || x_grid >= width || y_grid < 0 || y_grid >= height) {
      continue;
    }

    costmap[y_grid * width + x_grid] = max_cost;

    //inflate the obstacle
    int inflation = (int)(range / resolution);
    for (int i = -inflation; i < inflation; i++) {
      for (int j = -inflation; j < inflation; j++) {
        int x_grid_inflation = x_grid + i;
        int y_grid_inflation = y_grid + j;
        if (x_grid_inflation < 0 || x_grid_inflation >= width || y_grid_inflation < 0 || y_grid_inflation >= height) {
          continue;
        }
        double dis = std::sqrt(i*i + j*j)*resolution;
        int index = y_grid_inflation * width + x_grid_inflation;
        costmap[index]  = std::max(costmap[index], static_cast<int8_t>(std::max(0,100-(int)(dis/inflation_radius*100))));
      }
    }
  }
  
  // Create and publish the occupancy grid message
  nav_msgs::msg::OccupancyGrid costmap_msg;
  costmap_msg.header.stamp = msg->header.stamp;
  costmap_msg.header.frame_id = msg->header.frame_id;
  costmap_msg.info.resolution = resolution;
  costmap_msg.info.width = width;
  costmap_msg.info.height = height;
  costmap_msg.data = costmap;
  costmap_msg.info.origin.position.x = -15;
  costmap_msg.info.origin.position.y = -15;
  
  costmap_pub_->publish(costmap_msg);
}
  
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}