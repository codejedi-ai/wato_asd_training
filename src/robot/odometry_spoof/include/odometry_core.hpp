#ifndef ODOMETRY_CORE_HPP_
#define ODOMETRY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

namespace robot
{

class OdometryCore {
  public:
    explicit OdometryCore(const rclcpp::Logger& logger);
    
    // Public methods for the node to call
    nav_msgs::msg::Odometry computeOdometry(const geometry_msgs::msg::TransformStamped& transform);
    void reset();

  private:
    rclcpp::Logger logger_;
    
    // State tracking
    bool has_last_transform_;
    rclcpp::Time last_time_;
    tf2::Vector3 last_position_;
    tf2::Quaternion last_orientation_;
    
    // Helper methods
    void updateState(const nav_msgs::msg::Odometry& odom);
    void computeVelocity(nav_msgs::msg::Odometry& odom) const;
    void computeLinearVelocity(nav_msgs::msg::Odometry& odom) const;
    void computeAngularVelocity(nav_msgs::msg::Odometry& odom) const;
    double computeTimeDifference(const rclcpp::Time& current_time) const;
    void setZeroVelocity(nav_msgs::msg::Odometry& odom) const;
};

}  

#endif  
