#include "control_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odom_ = msg; });
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) {
    if (!msg || msg->poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty path; clearing current path and stopping");
      cur_path_.reset();
      if (cmd_vel_pub_) {
        cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
      }
      return;
    }
    cur_path_ = msg;
    RCLCPP_INFO(this->get_logger(), "New path received with %zu poses", msg->poses.size());
  });
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10, [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { costmap_ = msg; });
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(dt_), [this]() { this->controlLoop(); });
}
void ControlNode::controlLoop() {
  if (!cur_path_ || !odom_) {
    RCLCPP_ERROR(this->get_logger(), "No path or odom received");
    return;
  }
  if (cur_path_->poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "Current path has 0 poses; stopping");
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    cur_path_.reset();
    return;
  }
  double dis = computeDis(odom_->pose.pose.position, cur_path_->poses.back().pose.position);
  if (dis < goal_tolerance_) {
    RCLCPP_INFO(this->get_logger(), "Goal reached, stopping robot and clearing path");
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    cur_path_.reset(); // Clear the path
    return;
  }
  auto lookahead = findLookahead();
  if (dis < lookahead_distance_) {
    lookahead = cur_path_->poses.back();
  } else if (!lookahead.has_value()) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    return;
  }
  auto cmd = computeVel(lookahead.value());
  
  if (costmap_ && isNearObstacle(odom_->pose.pose.position.x, odom_->pose.pose.position.y, 0.3)) {
    cmd.linear.x *= 0.7; // Reduce speed to 70% when near obstacles
    RCLCPP_WARN(this->get_logger(), "Near obstacle, reducing speed to %.2f", cmd.linear.x);
  }
  
  cmd_vel_pub_->publish(cmd);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookahead() {
  for (auto& pose : cur_path_->poses) {
    double dis = computeDis(odom_->pose.pose.position, pose.pose.position);
    if (dis >= lookahead_distance_) {
      return pose;
    }
  }
  return std::nullopt;
}

double ControlNode::computeDis(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

bool ControlNode::isNearObstacle(double x, double y, double threshold) {
  if (!costmap_) return false;
  
  // Convert world coordinates to costmap grid coordinates
  double origin_x = costmap_->info.origin.position.x;
  double origin_y = costmap_->info.origin.position.y;
  double resolution = costmap_->info.resolution;
  
  int grid_x = static_cast<int>((x - origin_x) / resolution);
  int grid_y = static_cast<int>((y - origin_y) / resolution);
  
  if (grid_x < 0 || grid_x >= static_cast<int>(costmap_->info.width) ||
      grid_y < 0 || grid_y >= static_cast<int>(costmap_->info.height)) {
    return true; 
  }
  
  int radius = static_cast<int>(threshold / resolution);
  for (int dy = -radius; dy <= radius; dy++) {
    for (int dx = -radius; dx <= radius; dx++) {
      int check_x = grid_x + dx;
      int check_y = grid_y + dy;
      
      if (check_x >= 0 && check_x < static_cast<int>(costmap_->info.width) &&
          check_y >= 0 && check_y < static_cast<int>(costmap_->info.height)) {
        int index = check_y * costmap_->info.width + check_x;
        if (index < static_cast<int>(costmap_->data.size())) {
          int8_t cost = costmap_->data[index];

          if (cost > 50) {
            return true;
          }
        }
      }
    }
  }
  
  return false;
}

geometry_msgs::msg::Twist ControlNode::computeVel(const geometry_msgs::msg::PoseStamped& target) {
  double dx = target.pose.position.x - odom_->pose.pose.position.x;
  double dy = target.pose.position.y - odom_->pose.pose.position.y;
  tf2::Quaternion quat(odom_->pose.pose.orientation.x, odom_->pose.pose.orientation.y, odom_->pose.pose.orientation.z, odom_->pose.pose.orientation.w);
  tf2::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  double target_yaw = std::atan2(dy, dx);
  double yaw_error = target_yaw - yaw;

  while (yaw_error > M_PI) {
    yaw_error -= 2 * M_PI;
  }
  while (yaw_error < -M_PI) {
    yaw_error += 2 * M_PI;
  }

  double dis = computeDis(odom_->pose.pose.position, cur_path_->poses.back().pose.position);
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = std::min(lin_kp_ * dis, max_lin_vel_);
  cmd.angular.z = target == cur_path_->poses.back() ? 0 : ang_kp_ * yaw_error;
  return cmd;
}



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
