#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <chrono>

// ------------------- Supporting Structures -------------------

// 2D grid index
struct CellIndex
{
  int x;
  int y;
 
  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}
 
  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }
 
  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};
 
// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    // A simple hash combining x and y
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};
 
// Structure representing a node in the A* open set
struct AStarNode
{
  CellIndex index;
  double f_score;  // f = g + h
 
  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};
 
// Comparator for the priority queue (min-heap by f_score)
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    // We want the node with the smallest f_score on top
    return a.f_score > b.f_score;
  }
};

class PlannerNode : public rclcpp::Node {
  public:
      PlannerNode() : Node("planner_node"), state_(State::WAITING_FOR_GOAL) {
          // Subscribers
          map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
              "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
          goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
              "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
          odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
              "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
   
          // Publisher
          path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
   
          // Timer
          timer_ = this->create_wall_timer(
              std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
          
          RCLCPP_INFO(this->get_logger(), "Planner Node started");
      }
   
  private:
      enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };
      State state_;
   
      // Subscribers and Publisher
      rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
      rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
      rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
      rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
      rclcpp::TimerBase::SharedPtr timer_;
   
      // Data Storage
      nav_msgs::msg::OccupancyGrid current_map_;
      geometry_msgs::msg::PointStamped goal_;
      geometry_msgs::msg::Pose robot_pose_;
      nav_msgs::msg::Path current_path_;
   
      bool goal_received_ = false;
      bool has_map_ = false;
      bool has_goal_ = false;
      bool has_odom_ = false;
      
      // Map parameters
      int map_width_ = 300;
      int map_height_ = 300;
      double map_resolution_ = 0.1;
      double map_origin_x_ = 150.0;
      double map_origin_y_ = 150.0;
      
      // Timing
      std::chrono::steady_clock::time_point goal_received_time_;
      std::chrono::steady_clock::time_point last_plan_time_;
      double timeout_duration_ = 30.0; // 30 seconds timeout
   
      void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          current_map_ = *msg;
          has_map_ = true;
          
          // Update map parameters
          map_width_ = msg->info.width;
          map_height_ = msg->info.height;
          map_resolution_ = msg->info.resolution;
          map_origin_x_ = msg->info.origin.position.x;
          map_origin_y_ = msg->info.origin.position.y;
          
          RCLCPP_INFO(this->get_logger(), "Received map: %dx%d", map_width_, map_height_);
          
          if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
              planPath();
          }
      }
   
      void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
          goal_ = *msg;
          goal_received_ = true;
          has_goal_ = true;
          goal_received_time_ = std::chrono::steady_clock::now();
          state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
          
          RCLCPP_INFO(this->get_logger(), "Received goal at (%.2f, %.2f)", 
                      msg->point.x, msg->point.y);
          
          if (has_map_ && has_odom_) {
              planPath();
          }
      }
   
      void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
          robot_pose_ = msg->pose.pose;
          has_odom_ = true;
      }
   
      void timerCallback() {
          if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
              if (goalReached()) {
                  RCLCPP_INFO(this->get_logger(), "Goal reached!");
                  state_ = State::WAITING_FOR_GOAL;
              } else if (shouldReplan()) {
                  RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
                  planPath();
              }
          }
      }
   
      bool goalReached() {
          if (!has_odom_ || !has_goal_) {
              return false;
          }
          
          double dx = goal_.point.x - robot_pose_.position.x;
          double dy = goal_.point.y - robot_pose_.position.y;
          double distance = std::sqrt(dx * dx + dy * dy);
          
          double goal_tolerance = 0.5; // 0.5 meter tolerance
          return distance < goal_tolerance;
      }
      
      bool shouldReplan() {
          if (!has_odom_ || !has_goal_ || !has_map_) {
              return false;
          }
          
          // Check for timeout
          auto now = std::chrono::steady_clock::now();
          auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - goal_received_time_);
          
          if (elapsed.count() > timeout_duration_) {
              RCLCPP_WARN(this->get_logger(), "Goal timeout reached");
              state_ = State::WAITING_FOR_GOAL;
              return false;
          }
          
          // Check if enough time has passed since last plan
          auto time_since_last_plan = std::chrono::duration_cast<std::chrono::seconds>(now - last_plan_time_);
          return time_since_last_plan.count() >= 1; // Replan every 1 second
      }
   
      void planPath() {
          if (!has_map_ || !has_goal_ || !has_odom_) {
              RCLCPP_WARN(this->get_logger(), "Cannot plan path - missing data");
              return;
          }
          
          RCLCPP_INFO(this->get_logger(), "Planning path...");
          
          // Convert occupancy grid to 2D vector
          std::vector<std::vector<signed char>> map_2d(map_height_, std::vector<signed char>(map_width_));
          for (int y = 0; y < map_height_; ++y) {
              for (int x = 0; x < map_width_; ++x) {
                  map_2d[y][x] = current_map_.data[y * map_width_ + x];
              }
          }
          
          // Convert start and goal to grid coordinates
          CellIndex start_cell = worldToGrid(robot_pose_.position.x, robot_pose_.position.y);
          CellIndex goal_cell = worldToGrid(goal_.point.x, goal_.point.y);
          
          RCLCPP_INFO(this->get_logger(), "Planning from (%d, %d) to (%d, %d)", 
                      start_cell.x, start_cell.y, goal_cell.x, goal_cell.y);
          
          // Run A* algorithm
          std::vector<geometry_msgs::msg::PoseStamped> path_poses = aStar(map_2d, start_cell, goal_cell);
          
          if (path_poses.empty()) {
              RCLCPP_ERROR(this->get_logger(), "Failed to plan path");
              state_ = State::WAITING_FOR_GOAL;
          } else {
              RCLCPP_INFO(this->get_logger(), "Path planned successfully with %zu waypoints", path_poses.size());
              
              // Create path message
              current_path_.header.stamp = this->now();
              current_path_.header.frame_id = "map";
              current_path_.poses = path_poses;
              
              // Publish the path
              path_pub_->publish(current_path_);
              last_plan_time_ = std::chrono::steady_clock::now();
          }
      }
      
      // A* Algorithm Implementation
      std::vector<geometry_msgs::msg::PoseStamped> aStar(const std::vector<std::vector<signed char>>& map,
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
          
          RCLCPP_WARN(this->get_logger(), "A* failed to find a path");
          return {};
      }
      
      double heuristic(const CellIndex& a, const CellIndex& b) {
          // Euclidean distance heuristic
          double dx = a.x - b.x;
          double dy = a.y - b.y;
          return std::sqrt(dx * dx + dy * dy);
      }
      
      double gScore(const CellIndex& current, const CellIndex& neighbor) {
          // Calculate movement cost (diagonal movement costs more)
          int dx = std::abs(neighbor.x - current.x);
          int dy = std::abs(neighbor.y - current.y);
          
          if (dx == 1 && dy == 1) {
              return 1.414; // Diagonal movement
          } else {
              return 1.0; // Straight movement
          }
      }
      
      std::vector<CellIndex> getNeighbors(const CellIndex& cell, const std::vector<std::vector<signed char>>& map) {
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
      
      bool isValidCell(const CellIndex& cell, const std::vector<std::vector<signed char>>& /* map */) {
          return cell.x >= 0 && cell.x < map_width_ && cell.y >= 0 && cell.y < map_height_;
      }
      
      bool isObstacle(const CellIndex& cell, const std::vector<std::vector<signed char>>& map) {
          if (!isValidCell(cell, map)) return true;
          // Use occupancy values: -1 = unknown, 0 = free, 100 = occupied
          // Treat unknown cells as obstacles for safety
          signed char cell_value = map[cell.y][cell.x];
          return cell_value == -1 || cell_value == 100;
      }
      
      // Coordinate conversion functions
      CellIndex worldToGrid(double world_x, double world_y) const {
          // Map origin is at (150, 150) in grid coordinates, representing (0, 0) in world coordinates
          int x = static_cast<int>(world_x / map_resolution_) + 150;
          int y = static_cast<int>(world_y / map_resolution_) + 150;
          return CellIndex(x, y);
      }
      
      geometry_msgs::msg::PoseStamped gridToWorld(const CellIndex& cell) {
          geometry_msgs::msg::PoseStamped pose;
          pose.header.frame_id = "map";
          pose.header.stamp = this->now();
          
          // Convert from grid coordinates to world coordinates
          // Grid center (150, 150) represents world origin (0, 0)
          pose.pose.position.x = (cell.x - 150) * map_resolution_;
          pose.pose.position.y = (cell.y - 150) * map_resolution_;
          pose.pose.position.z = 0.0;
          
          pose.pose.orientation.w = 1.0;
          
          return pose;
      }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}