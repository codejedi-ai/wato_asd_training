#include "planner_node.hpp"

PlannerNode::PlannerNode() 
  : Node("planner"), 
    current_state_(PlannerState::WAITING_FOR_GOAL),
    has_map_(false),
    has_goal_(false),
    has_odom_(false),
    timeout_duration_(30.0), // 30 seconds timeout
    replan_interval_(1.0),   // 1 second replan interval
    planner_(robot::PlannerCore(this->get_logger()))
{
  // Create subscribers
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10,
    std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
    
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10,
    std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
    
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  
  // Create publisher
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  
  // Create timer for replanning
  replan_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(replan_interval_ * 1000)),
    std::bind(&PlannerNode::replanTimerCallback, this));
  
  RCLCPP_INFO(this->get_logger(), "Planner Node started");
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
  current_map_ = map;
  has_map_ = true;
  RCLCPP_INFO(this->get_logger(), "Received map");
  
  // If we have all required data, plan the path
  if (has_goal_ && has_odom_) {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr goal) {
  current_goal_ = *goal;
  has_goal_ = true;
  goal_received_time_ = std::chrono::steady_clock::now();
  current_state_ = PlannerState::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  
  RCLCPP_INFO(this->get_logger(), "Received goal at (%.2f, %.2f)", 
              goal->point.x, goal->point.y);
  
  // If we have all required data, plan the path
  if (has_map_ && has_odom_) {
    planPath();
  }
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
  current_odom_ = odom;
  has_odom_ = true;
}

void PlannerNode::replanTimerCallback() {
  if (current_state_ == PlannerState::WAITING_FOR_GOAL) {
    return; // Nothing to do in waiting state
  }
  
  // Check if goal is reached
  if (isGoalReached()) {
    RCLCPP_INFO(this->get_logger(), "Goal reached!");
    current_state_ = PlannerState::WAITING_FOR_GOAL;
    return;
  }
  
  // Check if we should replan
  if (shouldReplan()) {
    RCLCPP_INFO(this->get_logger(), "Replanning...");
    planPath();
  }
}

void PlannerNode::planPath() {
  if (!has_map_ || !has_goal_ || !has_odom_) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path - missing data");
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "Planning path...");
  
  // Plan the path using the planner core
  current_path_ = planner_.planPath(current_map_, current_goal_, current_odom_);
  
  if (current_path_.poses.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to plan path");
    current_state_ = PlannerState::WAITING_FOR_GOAL;
  } else {
    RCLCPP_INFO(this->get_logger(), "Path planned successfully with %zu waypoints", current_path_.poses.size());
    
    // Publish the path
    path_pub_->publish(current_path_);
    last_plan_time_ = std::chrono::steady_clock::now();
  }
}

bool PlannerNode::isGoalReached() {
  if (!has_odom_ || !has_goal_) {
    return false;
  }
  
  double dx = current_odom_->pose.pose.position.x - current_goal_.point.x;
  double dy = current_odom_->pose.pose.position.y - current_goal_.point.y;
  double distance = std::sqrt(dx * dx + dy * dy);
  
  double goal_tolerance = 0.5; // 0.5 meter tolerance
  return distance < goal_tolerance;
}

bool PlannerNode::shouldReplan() {
  if (!has_odom_ || !has_goal_ || !has_map_) {
    return false;
  }
  
  // Check for timeout
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - goal_received_time_);
  
  if (elapsed.count() > timeout_duration_) {
    RCLCPP_WARN(this->get_logger(), "Goal timeout reached");
    current_state_ = PlannerState::WAITING_FOR_GOAL;
    return false;
  }
  
  // Check if enough time has passed since last plan
  auto time_since_last_plan = std::chrono::duration_cast<std::chrono::seconds>(now - last_plan_time_);
  return time_since_last_plan.count() >= replan_interval_;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
