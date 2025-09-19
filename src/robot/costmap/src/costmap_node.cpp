#include "costmap_node.hpp"

// Static member definitions
const double CostmapNode::grid_resolution_ = 0.1;  // 0.1 meters per cell
const double CostmapNode::grid_origin_x_ = 150;    // Grid origin at (150,150) - robot position
const double CostmapNode::grid_origin_y_ = 150;    // Grid origin at (150,150) - robot position
const double CostmapNode::inflation_radius_ = 1.0; // 1 meter inflation radius

CostmapNode::CostmapNode() : Node("costmap") {
  // Initialize the grid
  grid_.resize(grid_height_, std::vector<signed char>(grid_width_, 0));
  
  // Create subscribers and publishers
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  
  RCLCPP_INFO(this->get_logger(), "Costmap Node started with %dx%d grid", grid_width_, grid_height_);
}
 
void CostmapNode::initializeCostmap() {
    // Reset all cells to free space (0)
    for (int y = 0; y < grid_height_; ++y) {
        for (int x = 0; x < grid_width_; ++x) {
            grid_[y][x] = 0;
        }
    }
}

void CostmapNode::convertToGrid(double range, double angle, int& x_grid, int& y_grid) {
    // Convert polar to Cartesian coordinates (in meters)
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);
    
    // Convert from meters to grid coordinates
    // Grid origin (150, 150) represents world (0, 0)
    // Convert meters to grid cells and add to origin
    x_grid = static_cast<int>(std::round(x / grid_resolution_)) + grid_origin_x_;
    y_grid = static_cast<int>(std::round(y / grid_resolution_)) + grid_origin_y_;
    
    // Assert positive values and within grid bounds
    assert(x_grid >= 0 && x_grid < 300 && "x_grid must be non negative and under 300");
    assert(y_grid >= 0 && y_grid < 300 && "y_grid must be non negative and under 300");
}

void CostmapNode::markObstacle(int x_grid, int y_grid) {
    // Check bounds and mark obstacle
    if (x_grid >= 0 && x_grid < grid_width_ && y_grid >= 0 && y_grid < grid_height_) {
        grid_[y_grid][x_grid] = 100; // 100 = occupied
    }
}

void CostmapNode::inflateObstacles() {
    // Create a copy for inflation
    std::vector<std::vector<signed char>> inflated_grid = grid_;
    int cell_radius = static_cast<int>(inflation_radius_ / grid_resolution_);
    
    for (int y = 0; y < grid_height_; ++y) {
        for (int x = 0; x < grid_width_; ++x) {
            if (grid_[y][x] == 100) { // If cell is occupied
                // Inflate around this obstacle
                for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
                    for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;
                        
                        if (nx >= 0 && nx < grid_width_ && ny >= 0 && ny < grid_height_) {
                            double distance = std::sqrt(dx * dx + dy * dy) * grid_resolution_;
                            if (distance <= inflation_radius_) {
                                // Calculate cost based on distance
                                signed char cost = static_cast<signed char>(100 * (1.0 - distance / inflation_radius_));
                                if (inflated_grid[ny][nx] < cost) {
                                    inflated_grid[ny][nx] = cost;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    
    grid_ = inflated_grid;
}

void CostmapNode::publishCostmap() {
    nav_msgs::msg::OccupancyGrid costmap_msg;
    
    // Set header
    costmap_msg.header.stamp = this->now();
    costmap_msg.header.frame_id = "map";
    
    // Set map metadata
    costmap_msg.info.resolution = grid_resolution_;
    costmap_msg.info.width = grid_width_;
    costmap_msg.info.height = grid_height_;
    costmap_msg.info.origin.position.x = grid_origin_x_;  // World origin in meters the robot is seeing the world as if it si 0 0
    costmap_msg.info.origin.position.y = grid_origin_y_;  // World origin in meters
    costmap_msg.info.origin.position.z = 0.0;
    costmap_msg.info.origin.orientation.w = 0.0;
    
    // Flatten 2D grid to 1D array (row-major order)
    costmap_msg.data.resize(grid_width_ * grid_height_);
    for (int y = 0; y < grid_height_; ++y) {
        for (int x = 0; x < grid_width_; ++x) {
            costmap_msg.data[y * grid_width_ + x] = grid_[y][x];
        }
    }
    
    // Publish the costmap
    costmap_pub_->publish(costmap_msg);
}

void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // Step 1: Initialize costmap
    initializeCostmap();
 
    // Step 2: Convert LaserScan to grid and mark obstacles
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];
        
        // Check if range is valid
        if (std::isfinite(range) && range >= scan->range_min && range <= scan->range_max) {
            // Calculate grid coordinates
            int x_grid, y_grid;
            convertToGrid(range, angle, x_grid, y_grid);
            markObstacle(x_grid, y_grid);
        }
    }
 
    // Step 3: Inflate obstacles
    inflateObstacles();
 
    // Step 4: Publish costmap
    publishCostmap();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}