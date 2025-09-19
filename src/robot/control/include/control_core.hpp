#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>
#include <cmath>
#include <optional>

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);
    
    // Public methods
    geometry_msgs::msg::Twist computeControl(const nav_msgs::msg::Path::SharedPtr path,
                                           const nav_msgs::msg::Odometry::SharedPtr odom);
    void setLookaheadDistance(double distance);
    void setLinearSpeed(double speed);
    void setGoalTolerance(double tolerance);

  private:
    rclcpp::Logger logger_;
    
    // Control parameters
    double lookahead_distance_;
    double linear_speed_;
    double goal_tolerance_;
    double max_angular_velocity_;
    double control_gain_;
    
    // Helper methods
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(
        const std::vector<geometry_msgs::msg::PoseStamped>& path_poses,
        const geometry_msgs::msg::PoseStamped& robot_pose);
    double computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b);
    double extractYaw(const geometry_msgs::msg::Quaternion& quat);
    double computePurePursuitSteeringAngle(const geometry_msgs::msg::PoseStamped& robot_pose,
                                          const geometry_msgs::msg::PoseStamped& target_pose);
    bool isGoalReached(const geometry_msgs::msg::PoseStamped& robot_pose,
                      const geometry_msgs::msg::PoseStamped& goal_pose);
    double normalizeAngle(double angle);
    int findClosestPointIndex(const std::vector<geometry_msgs::msg::PoseStamped>& path_poses,
                             const geometry_msgs::msg::PoseStamped& robot_pose);
};

} 

#endif 
