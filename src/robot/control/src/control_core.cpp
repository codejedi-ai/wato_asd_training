#include "control_core.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) 
: logger_(logger),
  lookahead_distance_(1.0),
  goal_tolerance_(0.1),
  linear_speed_(0.5),
  max_angular_velocity_(1.0)
{
  RCLCPP_INFO(logger_, "Control Core initialized");
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(const nav_msgs::msg::Path::SharedPtr path,
                                                      const nav_msgs::msg::Odometry::SharedPtr odom) {
  if (!path || !odom) {
    geometry_msgs::msg::Twist cmd_vel;
    return cmd_vel; // Return zero velocity
  }
  
  // Find the lookahead point
  auto lookahead_point = findLookaheadPoint(path, odom);
  if (!lookahead_point) {
    geometry_msgs::msg::Twist cmd_vel;
    return cmd_vel; // Return zero velocity
  }
  
  // Compute velocity command
  return computeVelocityCommand(*lookahead_point, odom);
}

void ControlCore::setParameters(double lookahead_distance, double goal_tolerance, double linear_speed) {
  lookahead_distance_ = lookahead_distance;
  goal_tolerance_ = goal_tolerance;
  linear_speed_ = linear_speed;
  
  RCLCPP_INFO(logger_, "Control parameters updated: lookahead=%.2f, tolerance=%.2f, speed=%.2f",
              lookahead_distance_, goal_tolerance_, linear_speed_);
}

bool ControlCore::isGoalReached(const nav_msgs::msg::Path::SharedPtr path,
                               const nav_msgs::msg::Odometry::SharedPtr odom) const {
  if (!path || !odom || path->poses.empty()) {
    return false;
  }
  
  double distance_to_goal = computeDistance(odom->pose.pose.position, path->poses.back().pose.position);
  return distance_to_goal <= goal_tolerance_;
}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(const nav_msgs::msg::Path::SharedPtr path,
                                                                              const nav_msgs::msg::Odometry::SharedPtr odom) const {
  if (!path || path->poses.empty() || !odom) {
    return std::nullopt;
  }
  
  // Find the closest point on the path to the robot
  int closest_index = findClosestPointIndex(path, odom);
  
  // Search forward from the closest point for the lookahead point
  for (int i = closest_index; i < static_cast<int>(path->poses.size()); ++i) {
    double distance = computeDistance(odom->pose.pose.position, path->poses[i].pose.position);
    if (distance >= lookahead_distance_) {
      return path->poses[i];
    }
  }
  
  // If no point is far enough, return the last point
  return path->poses.back();
}

geometry_msgs::msg::Twist ControlCore::computeVelocityCommand(const geometry_msgs::msg::PoseStamped& target,
                                                             const nav_msgs::msg::Odometry::SharedPtr odom) const {
  geometry_msgs::msg::Twist cmd_vel;
  
  // Check if goal is reached
  if (isGoalReached(nullptr, odom)) {
    // Stop the robot
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    return cmd_vel;
  }
  
  // Extract robot yaw
  double robot_yaw = extractYaw(odom->pose.pose.orientation);
  
  // Compute vector to target
  double dx = target.pose.position.x - odom->pose.pose.position.x;
  double dy = target.pose.position.y - odom->pose.pose.position.y;
  
  // Compute angle to target relative to robot's current heading
  double alpha = std::atan2(dy, dx) - robot_yaw;
  
  // Normalize angle to [-π, π]
  while (alpha > M_PI) alpha -= 2.0 * M_PI;
  while (alpha < -M_PI) alpha += 2.0 * M_PI;
  
  // Pure Pursuit steering angle calculation: steering_angle = 2 * sin(alpha) / L
  double steering_angle = 2.0 * std::sin(alpha) / lookahead_distance_;
  
  // Limit angular velocity
  if (steering_angle > max_angular_velocity_) steering_angle = max_angular_velocity_;
  if (steering_angle < -max_angular_velocity_) steering_angle = -max_angular_velocity_;
  
  // Set velocities
  cmd_vel.linear.x = linear_speed_;
  cmd_vel.angular.z = steering_angle;
  
  return cmd_vel;
}

double ControlCore::computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) const {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& quat) const {
  double x = quat.x;
  double y = quat.y;
  double z = quat.z;
  double w = quat.w;
  
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

int ControlCore::findClosestPointIndex(const nav_msgs::msg::Path::SharedPtr path,
                                      const nav_msgs::msg::Odometry::SharedPtr odom) const {
  if (!path || path->poses.empty() || !odom) {
    return 0;
  }
  
  int closest_index = 0;
  double min_distance = computeDistance(odom->pose.pose.position, path->poses[0].pose.position);
  
  for (int i = 1; i < static_cast<int>(path->poses.size()); ++i) {
    double distance = computeDistance(odom->pose.pose.position, path->poses[i].pose.position);
    if (distance < min_distance) {
      min_distance = distance;
      closest_index = i;
    }
  }
  
  return closest_index;
}

} 