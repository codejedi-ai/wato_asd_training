#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>
#include <optional>

namespace robot
{

class ControlCore {
  public:
    explicit ControlCore(const rclcpp::Logger& logger);
    
    // Public methods for the node to call
    geometry_msgs::msg::Twist computeVelocity(const nav_msgs::msg::Path::SharedPtr path,
                                             const nav_msgs::msg::Odometry::SharedPtr odom);
    void setParameters(double lookahead_distance, double goal_tolerance, double linear_speed);
    bool isGoalReached(const nav_msgs::msg::Path::SharedPtr path,
                      const nav_msgs::msg::Odometry::SharedPtr odom) const;

  private:
    rclcpp::Logger logger_;
    
    // Control parameters
    double lookahead_distance_;
    double goal_tolerance_;
    double linear_speed_;
    double max_angular_velocity_;
    
    // Helper methods
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(const nav_msgs::msg::Path::SharedPtr path,
                                                                     const nav_msgs::msg::Odometry::SharedPtr odom) const;
    geometry_msgs::msg::Twist computeVelocityCommand(const geometry_msgs::msg::PoseStamped& target,
                                                    const nav_msgs::msg::Odometry::SharedPtr odom) const;
    double computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) const;
    double extractYaw(const geometry_msgs::msg::Quaternion& quat) const;
    int findClosestPointIndex(const nav_msgs::msg::Path::SharedPtr path,
                             const nav_msgs::msg::Odometry::SharedPtr odom) const;
};

}  

#endif  