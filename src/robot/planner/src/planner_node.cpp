#include "planner_node.hpp"
#include <vector>
#include <algorithm>
#include <sstream>
#include <iomanip>

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  RCLCPP_INFO(this->get_logger(), "Initializing PlannerNode");
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  RCLCPP_INFO(this->get_logger(), "PlannerNode initialized");
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "odomCallback");
  pose_msg_ = msg->pose.pose;
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "mapCallback (size=%zu)", msg->data.size());
  map_msg_ = *msg;
  if (map_msg_.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty map, cannot plan path!");
    return;
  }
  if (plan_flag_ == PlannerState::waiting_for_robot_to_reach_goal) {
    RCLCPP_INFO(this->get_logger(), "Map updated: triggering createPath");
    createPath();
  }
}
void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "goalCallback: (%.3f, %.3f)", msg->point.x, msg->point.y);
  goal_msg_ = *msg;
  plan_flag_ = PlannerState::waiting_for_robot_to_reach_goal;
  goal_flag_ = true;
  createPath();
}

void PlannerNode::timerCallback() {
  if (plan_flag_ == PlannerState::waiting_for_robot_to_reach_goal && goal_flag_) {
    if (goalReached()) {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      plan_flag_ = PlannerState::waiting_for_goal;
      nav_msgs::msg::Path empty_path;
      path_pub_->publish(empty_path);
    } else {
      createPath();
    }
  }
}

bool PlannerNode::goalReached() {
  double dx = goal_msg_.point.x - pose_msg_.position.x;
  double dy = goal_msg_.point.y - pose_msg_.position.y;
  return std::sqrt(dx * dx + dy * dy) < 0.5;
}

void PlannerNode::createPath() {
  if (!goal_flag_ || map_msg_.data.empty()) {
    return;
  }

  // Convert robot and goal positions to grid indices
  CellIndex start = getCellIndex(pose_msg_);
  geometry_msgs::msg::Pose goal_pose;
  goal_pose.position.x = goal_msg_.point.x;
  goal_pose.position.y = goal_msg_.point.y;
  goal_pose.orientation.w = 1.0;
  CellIndex goal = getCellIndex(goal_pose);

  bool found = false;
  int attempt = 0;
  nav_msgs::msg::Path path;
  obstacle_threshold_ = 50; 
  check_bound_ratio = 1.0;
  // A* algorithm
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;

  open_set.emplace(start, 0.0);
  g_score[start] = 0.0;

  while (!found && check_bound_ratio >= 0.2) {
    if (obstacle_threshold_ > 50) { 
      check_bound_ratio -= 0.2;
    }
    // reset for each attempt
    came_from.clear();
    g_score.clear();
    g_score[start] = 0.0;
    open_set = std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF>();
    // RCLCPP_INFO(this->get_logger(), "attempt %d with obstacle threshold %d", attempt + 1, obstacle_threshold_);

    open_set.emplace(start, 0.0);
    while (!open_set.empty()) {
      CellIndex current = open_set.top().index;
      if (current == goal) {
        found = true;
        break;
      }
      open_set.pop();

      // 8-connected neighbors
      std::vector<CellIndex> neighbors = {
        CellIndex(current.x + 1, current.y),
        CellIndex(current.x - 1, current.y),
        CellIndex(current.x, current.y + 1),
        CellIndex(current.x, current.y - 1),
        CellIndex(current.x + 1, current.y + 1),
        CellIndex(current.x - 1, current.y + 1),
        CellIndex(current.x + 1, current.y - 1),
        CellIndex(current.x - 1, current.y - 1),
      };

      for (const auto& neighbor : neighbors) {
        if (!checkValid(neighbor)) continue;
        double tentative_g = g_score[current] + 1.0; // cost between cells

        if (!g_score.count(neighbor) || tentative_g < g_score[neighbor]) {
          came_from[neighbor] = current;
          g_score[neighbor] = tentative_g;
          double f = tentative_g + heuristic(goal, neighbor);
          open_set.emplace(neighbor, f);
        }
      }
    }

    if (!found) {
      obstacle_threshold_ += THRESHOLD_INCREMENT;
      attempt++;
    }
  }

  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "sim_world";
  if (found) {
    std::vector<CellIndex> cells;
    CellIndex current = goal;
    while (current != start) {
      cells.push_back(current);
      current = came_from[current];
    }
    cells.push_back(start);
    std::reverse(cells.begin(), cells.end());

    // Visualize the path on the map by marking path cells with -1
    nav_msgs::msg::OccupancyGrid viz_map = map_msg_; // Copy the map
    for (const auto& cell : cells) {
      int idx = cell.y * viz_map.info.width + cell.x;
      if (idx >= 0 && idx < static_cast<int>(viz_map.data.size())) {
        viz_map.data[idx] = -1; // Mark path cell
      }
    }

    for (const auto& cell : cells) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose = getPose(cell);
      path.poses.push_back(pose);
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "No valid path found to goal");
    // No recovery path - publish empty path
  }
  path_pub_->publish(path);
}

bool PlannerNode::checkValid(const CellIndex& index) {
  RCLCPP_DEBUG(this->get_logger(), "checkValid (%d,%d)", index.x, index.y);
  // Bounds check for the cell itself
  if (index.x < 0 || index.x >= static_cast<int>(map_msg_.info.width) ||
      index.y < 0 || index.y >= static_cast<int>(map_msg_.info.height)) {
    return false;
  }

  // Robot footprint parameters (adaptive sizing)
  double ROBOT_LENGTH = 1.5 * check_bound_ratio;   // meters (behind the robot)
  double ROBOT_WIDTH  = 1.0 * check_bound_ratio;   // meters (centered on robot)
  constexpr double GRID_STEP    = 0.1;   // meters (resolution for checking)

  // Robot orientation (yaw)
  auto& q = pose_msg_.orientation;
  double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  // World coordinates of the given cell (treated as robot front)
  geometry_msgs::msg::Pose world_pose = getPose(index);
  double rx = world_pose.position.x;
  double ry = world_pose.position.y;

  // Check all points in the rectangle footprint
  for (double back = 0.0; back <= ROBOT_LENGTH; back += GRID_STEP) {
    for (double side = -ROBOT_WIDTH/2.0; side <= ROBOT_WIDTH/2.0; side += GRID_STEP) {
      // Compute the point in the robot frame (front is at 0,0)
      double local_x = -back;      // negative: behind the robot
      double local_y = side;

      // Transform to world frame
      double wx = rx + std::cos(yaw) * local_x - std::sin(yaw) * local_y;
      double wy = ry + std::sin(yaw) * local_x + std::cos(yaw) * local_y;

      // Convert to grid index
      int map_x = static_cast<int>(wx / map_msg_.info.resolution + static_cast<int>(map_msg_.info.width) / 2);
      int map_y = static_cast<int>(wy / map_msg_.info.resolution + static_cast<int>(map_msg_.info.height) / 2);

      // Check bounds
      if (map_x < 0 || map_x >= static_cast<int>(map_msg_.info.width) ||
          map_y < 0 || map_y >= static_cast<int>(map_msg_.info.height)) {
        return false; // Out of map bounds, treat as collision
      }

      int map_idx = map_y * map_msg_.info.width + map_x;
      int8_t cost = map_msg_.data[map_idx];
      // Treat high-cost cells as obstacles
      if (cost >= obstacle_threshold_) {
        return false; // Collision detected
      }
    }
  }
  return true; // All footprint cells are free
}

CellIndex PlannerNode::getCellIndex(const geometry_msgs::msg::Pose& pose) {
  RCLCPP_DEBUG(this->get_logger(), "getCellIndex");
  int x = static_cast<int>(pose.position.x/map_msg_.info.resolution + static_cast<int>(map_msg_.info.width/2));
  int y = static_cast<int>(pose.position.y/map_msg_.info.resolution + static_cast<int>(map_msg_.info.height/2));
  return CellIndex(x, y);
}

geometry_msgs::msg::Pose PlannerNode::getPose(const CellIndex& index) {
  RCLCPP_DEBUG(this->get_logger(), "getPose");
  geometry_msgs::msg::Pose pose;
  pose.position.x = (index.x - static_cast<int>(map_msg_.info.width/2)) * map_msg_.info.resolution;
  pose.position.y = (index.y - static_cast<int>(map_msg_.info.height/2)) * map_msg_.info.resolution;
  pose.orientation.w = 1.0;
  pose.orientation.z = 0.0;
  return pose;
}

double PlannerNode::heuristic(const CellIndex& goal, const CellIndex& current) {
  RCLCPP_DEBUG(this->get_logger(), "heuristic");
  return std::sqrt(std::pow(goal.x - current.x, 2) + std::pow(goal.y - current.y, 2));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
