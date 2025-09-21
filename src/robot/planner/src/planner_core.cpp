#include "planner_core.hpp"
#include <algorithm>
#include <functional>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) 
: logger_(logger),
  map_width_(300),
  map_height_(300),
  map_resolution_(0.1),
  map_origin_x_(150.0),
  map_origin_y_(150.0)
{
  RCLCPP_INFO(logger_, "Planner Core initialized");
}

nav_msgs::msg::Path PlannerCore::planPath(const nav_msgs::msg::OccupancyGrid::SharedPtr map,
                                         const geometry_msgs::msg::PointStamped& goal,
                                         const nav_msgs::msg::Odometry::SharedPtr odom) {
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp = rclcpp::Time();
  
  // Update map parameters
  map_width_ = map->info.width;
  map_height_ = map->info.height;
  map_resolution_ = map->info.resolution;
  map_origin_x_ = map->info.origin.position.x;
  map_origin_y_ = map->info.origin.position.y;
  
  // Convert occupancy grid to 2D vector
  std::vector<std::vector<signed char>> map_2d(map_height_, std::vector<signed char>(map_width_));
  for (int y = 0; y < map_height_; ++y) {
    for (int x = 0; x < map_width_; ++x) {
      map_2d[y][x] = map->data[y * map_width_ + x];
    }
  }
  
  // Convert start and goal to grid coordinates
  CellIndex start_cell = odomToGrid(odom);
  CellIndex goal_cell = pointToGrid(goal);
  
  RCLCPP_INFO(logger_, "Planning from (%d, %d) to (%d, %d)", 
              start_cell.x, start_cell.y, goal_cell.x, goal_cell.y);
  
  // Run A* algorithm
  std::vector<geometry_msgs::msg::PoseStamped> path_poses = aStar(map_2d, start_cell, goal_cell);
  
  if (path_poses.empty()) {
    RCLCPP_WARN(logger_, "No path found!");
    return path;
  }
  
  // Smooth the path
  path_poses = smoothPath(path_poses);
  
  path.poses = path_poses;
  RCLCPP_INFO(logger_, "Path planned with %zu waypoints", path_poses.size());
  
  return path;
}

bool PlannerCore::isGoalReached(const geometry_msgs::msg::PointStamped& goal,
                               const nav_msgs::msg::Odometry::SharedPtr odom,
                               double tolerance) const {
  double distance = calculateDistance(odom->pose.pose.position, goal.point);
  return distance < tolerance;
}

std::vector<geometry_msgs::msg::PoseStamped> PlannerCore::aStar(const std::vector<std::vector<signed char>>& map,
                                                               const CellIndex& start, const CellIndex& goal) {
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_set<CellIndex, CellIndexHash> closed_set;
  std::unordered_map<CellIndex, double, CellIndexHash> g_scores;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  
  // Initialize start node
  double start_f = heuristic(start, goal);
  open_set.push(AStarNode(start, start_f));
  g_scores[start] = 0.0;
  
  while (!open_set.empty()) {
    AStarNode current = open_set.top();
    open_set.pop();
    
    // Check if we reached the goal
    if (current.index == goal) {
      // Reconstruct path
      std::vector<geometry_msgs::msg::PoseStamped> path;
      CellIndex current_cell = goal;
      
      while (current_cell != start) {
        geometry_msgs::msg::PoseStamped pose = gridToWorld(current_cell);
        path.push_back(pose);
        current_cell = came_from[current_cell];
      }
      
      // Add start position
      geometry_msgs::msg::PoseStamped start_pose = gridToWorld(start);
      path.push_back(start_pose);
      
      std::reverse(path.begin(), path.end());
      return path;
    }
    
    // Skip if already processed
    if (closed_set.find(current.index) != closed_set.end()) {
      continue;
    }
    closed_set.insert(current.index);
    
    // Explore neighbors
    std::vector<CellIndex> neighbors = getNeighbors(current.index, map);
    for (const CellIndex& neighbor : neighbors) {
      if (closed_set.find(neighbor) != closed_set.end()) {
        continue;
      }
      
      double tentative_g = g_scores[current.index] + gScore(current.index, neighbor);
      
      // Check if this path to neighbor is better
      if (g_scores.find(neighbor) == g_scores.end() || tentative_g < g_scores[neighbor]) {
        g_scores[neighbor] = tentative_g;
        came_from[neighbor] = current.index;
        
        double f_score = tentative_g + heuristic(neighbor, goal);
        open_set.push(AStarNode(neighbor, f_score));
      }
    }
  }
  
  RCLCPP_WARN(logger_, "A* failed to find a path");
  return {};
}

double PlannerCore::heuristic(const CellIndex& a, const CellIndex& b) {
  // Euclidean distance heuristic
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

double PlannerCore::gScore(const CellIndex& current, const CellIndex& neighbor) {
  // Calculate movement cost (diagonal movement costs more)
  int dx = std::abs(neighbor.x - current.x);
  int dy = std::abs(neighbor.y - current.y);
  
  if (dx == 1 && dy == 1) {
    return 1.414; // Diagonal movement
  } else {
    return 1.0; // Straight movement
  }
}

std::vector<CellIndex> PlannerCore::getNeighbors(const CellIndex& cell, const std::vector<std::vector<signed char>>& map) {
  std::vector<CellIndex> neighbors;
  
  // 8-connected grid
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) continue;
      
      CellIndex neighbor(cell.x + dx, cell.y + dy);
      if (isValidCell(neighbor, map) && !isObstacle(neighbor, map)) {
        neighbors.push_back(neighbor);
      }
    }
  }
  
  return neighbors;
}

bool PlannerCore::isValidCell(const CellIndex& cell, const std::vector<std::vector<signed char>>& /* map */) {
  return cell.x >= 0 && cell.x < map_width_ && cell.y >= 0 && cell.y < map_height_;
}

bool PlannerCore::isObstacle(const CellIndex& cell, const std::vector<std::vector<signed char>>& map) {
  if (!isValidCell(cell, map)) return true;
  // Use occupancy values: -1 = unknown, 0 = free, 100 = occupied
  // Treat unknown cells as obstacles for safety
  signed char cell_value = map[cell.y][cell.x];
  return cell_value == -1 || cell_value == 100;
}

CellIndex PlannerCore::worldToGrid(double world_x, double world_y) const {
  // Map origin is at (150, 150) in grid coordinates, representing (0, 0) in world coordinates
  int x = static_cast<int>(world_x / map_resolution_) + 150;
  int y = static_cast<int>(world_y / map_resolution_) + 150;
  return CellIndex(x, y);
}

geometry_msgs::msg::PoseStamped PlannerCore::gridToWorld(const CellIndex& cell) const {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = rclcpp::Time();
  
  // Convert from grid coordinates to world coordinates
  // Grid center (150, 150) represents world origin (0, 0)
  pose.pose.position.x = (cell.x - 150) * map_resolution_;
  pose.pose.position.y = (cell.y - 150) * map_resolution_;
  pose.pose.position.z = 0.0;
  
  pose.pose.orientation.w = 1.0;
  
  return pose;
}

CellIndex PlannerCore::odomToGrid(const nav_msgs::msg::Odometry::SharedPtr odom) const {
  return worldToGrid(odom->pose.pose.position.x, odom->pose.pose.position.y);
}

CellIndex PlannerCore::pointToGrid(const geometry_msgs::msg::PointStamped& point) const {
  return worldToGrid(point.point.x, point.point.y);
}

std::vector<geometry_msgs::msg::PoseStamped> PlannerCore::smoothPath(const std::vector<geometry_msgs::msg::PoseStamped>& path) {
  if (path.size() <= 2) return path;
  
  std::vector<geometry_msgs::msg::PoseStamped> smoothed;
  smoothed.push_back(path[0]);
  
  // Simple path smoothing - keep every 3rd point
  for (size_t i = 3; i < path.size(); i += 3) {
    smoothed.push_back(path[i]);
  }
  
  // Always include the last point
  if (smoothed.back().pose.position.x != path.back().pose.position.x ||
      smoothed.back().pose.position.y != path.back().pose.position.y) {
    smoothed.push_back(path.back());
  }
  
  return smoothed;
}

double PlannerCore::calculateDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) const {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

} 
