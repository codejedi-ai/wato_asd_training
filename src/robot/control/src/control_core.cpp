#include "control_core.hpp"

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger),
    lookahead_distance_(1.0),
    linear_speed_(0.5),
    goal_tolerance_(0.1),
    max_angular_velocity_(1.0),
    control_gain_(1.0)
{
  RCLCPP_INFO(logger_, "Pure Pursuit Control Core initialized");
}

geometry_msgs::msg::Twist ControlCore::computeControl(const nav_msgs::msg::Path::SharedPtr path,
                                                     const nav_msgs::msg::Odometry::SharedPtr odom) {
  geometry_msgs::msg::Twist cmd_vel;
  
  if (!path || path->poses.empty() || !odom) {
    RCLCPP_WARN(logger_, "Missing path or odometry data");
    return cmd_vel;
  }
  
  // Create robot pose from odometry
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header = odom->header;
  robot_pose.pose = odom->pose.pose;
  
  // Check if goal is reached
  if (isGoalReached(robot_pose, path->poses.back())) {
    RCLCPP_INFO(logger_, "Goal reached!");
    return cmd_vel; // Stop the robot
  }
  
  // Find lookahead point using Pure Pursuit algorithm
  auto lookahead_point = findLookaheadPoint(path->poses, robot_pose);
  if (!lookahead_point) {
    RCLCPP_WARN(logger_, "No valid lookahead point found");
    return cmd_vel;
  }
  
  // Compute Pure Pursuit steering angle
  double steering_angle = computePurePursuitSteeringAngle(robot_pose, *lookahead_point);
  
  // Set linear and angular velocities
  cmd_vel.linear.x = linear_speed_;
  cmd_vel.angular.z = steering_angle;
  
  RCLCPP_DEBUG(logger_, "Control: linear=%.2f, angular=%.2f", cmd_vel.linear.x, cmd_vel.angular.z);
  
  return cmd_vel;
}

void ControlCore::setLookaheadDistance(double distance) {
  lookahead_distance_ = distance;
  RCLCPP_INFO(logger_, "Lookahead distance set to %.2f", distance);
}

void ControlCore::setLinearSpeed(double speed) {
  linear_speed_ = speed;
  RCLCPP_INFO(logger_, "Linear speed set to %.2f", speed);
}

void ControlCore::setGoalTolerance(double tolerance) {
  goal_tolerance_ = tolerance;
  RCLCPP_INFO(logger_, "Goal tolerance set to %.2f", tolerance);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(
    const std::vector<geometry_msgs::msg::PoseStamped>& path_poses,
    const geometry_msgs::msg::PoseStamped& robot_pose) {
  
  if (path_poses.empty()) {
    return std::nullopt;
  }
  
  // Find the closest point on the path to the robot
  int closest_index = findClosestPointIndex(path_poses, robot_pose);
  
  // Search forward from the closest point for the lookahead point
  for (int i = closest_index; i < static_cast<int>(path_poses.size()); ++i) {
    double distance = computeDistance(robot_pose.pose.position, path_poses[i].pose.position);
    if (distance >= lookahead_distance_) {
      return path_poses[i];
    }
  }
  
  // If no point is far enough, return the last point
  return path_poses.back();
}

double ControlCore::computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& quat) {
  double x = quat.x;
  double y = quat.y;
  double z = quat.z;
  double w = quat.w;
  
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

double ControlCore::computePurePursuitSteeringAngle(const geometry_msgs::msg::PoseStamped& robot_pose,
                                                   const geometry_msgs::msg::PoseStamped& target_pose) {
  // Extract robot yaw
  double robot_yaw = extractYaw(robot_pose.pose.orientation);
  
  // Compute vector to target
  double dx = target_pose.pose.position.x - robot_pose.pose.position.x;
  double dy = target_pose.pose.position.y - robot_pose.pose.position.y;
  
  // Compute distance to target
  double distance = std::sqrt(dx * dx + dy * dy);
  
  // Pure Pursuit algorithm: steering angle = 2 * sin(alpha) / L
  // where alpha is the angle between robot heading and line to target
  // and L is the lookahead distance
  
  // Compute angle to target relative to robot's current heading
  double alpha = std::atan2(dy, dx) - robot_yaw;
  alpha = normalizeAngle(alpha);
  
  // Pure Pursuit steering angle calculation
  double steering_angle = 2.0 * std::sin(alpha) / lookahead_distance_;
  
  // Apply control gain
  steering_angle *= control_gain_;
  
  // Limit angular velocity
  if (steering_angle > max_angular_velocity_) steering_angle = max_angular_velocity_;
  if (steering_angle < -max_angular_velocity_) steering_angle = -max_angular_velocity_;
  
  return steering_angle;
}

bool ControlCore::isGoalReached(const geometry_msgs::msg::PoseStamped& robot_pose,
                               const geometry_msgs::msg::PoseStamped& goal_pose) {
  double distance = computeDistance(robot_pose.pose.position, goal_pose.pose.position);
  return distance <= goal_tolerance_;
}

double ControlCore::normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

int ControlCore::findClosestPointIndex(const std::vector<geometry_msgs::msg::PoseStamped>& path_poses,
                                      const geometry_msgs::msg::PoseStamped& robot_pose) {
  if (path_poses.empty()) {
    return -1;
  }
  
  int closest_index = 0;
  double min_distance = computeDistance(robot_pose.pose.position, path_poses[0].pose.position);
  
  for (int i = 1; i < static_cast<int>(path_poses.size()); ++i) {
    double distance = computeDistance(robot_pose.pose.position, path_poses[i].pose.position);
    if (distance < min_distance) {
      min_distance = distance;
      closest_index = i;
    }
  }
  
  return closest_index;
}

}  
