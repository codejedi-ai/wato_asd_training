#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <chrono>

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    // State machine states
    enum class PlannerState {
      WAITING_FOR_GOAL,
      WAITING_FOR_ROBOT_TO_REACH_GOAL
    };
    
    // Callback functions
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr goal);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
    void replanTimerCallback();
    
    // Planning functions
    void planPath();
    bool isGoalReached();
    bool shouldReplan();
    
    // ROS constructs
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr replan_timer_;
    
    // State machine
    PlannerState current_state_;
    
    // Data storage
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    geometry_msgs::msg::PointStamped current_goal_;
    nav_msgs::msg::Odometry::SharedPtr current_odom_;
    nav_msgs::msg::Path current_path_;
    
    // Flags
    bool has_map_;
    bool has_goal_;
    bool has_odom_;
    
    // Timing
    std::chrono::steady_clock::time_point goal_received_time_;
    std::chrono::steady_clock::time_point last_plan_time_;
    double timeout_duration_; // seconds
    double replan_interval_; // seconds
    
    // Core functionality
    robot::PlannerCore planner_;
};

#endif 
