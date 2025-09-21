#include "odometry_core.hpp"

namespace robot
{

OdometryCore::OdometryCore(const rclcpp::Logger& logger) 
: logger_(logger),
  has_last_transform_(false)
{
  RCLCPP_INFO(logger_, "Odometry Core initialized");
}

nav_msgs::msg::Odometry OdometryCore::computeOdometry(const geometry_msgs::msg::TransformStamped& transform) {
  // Create an Odometry message
  nav_msgs::msg::Odometry odom_msg;

  // Fill header
  odom_msg.header.stamp = transform.header.stamp;
  odom_msg.header.frame_id = "map";  // Use "map" frame for consistency
  odom_msg.child_frame_id  = "base_link";  // Standard robot base frame

  // Pose from TF
  odom_msg.pose.pose.position.x = transform.transform.translation.x;
  odom_msg.pose.pose.position.y = transform.transform.translation.y;
  odom_msg.pose.pose.position.z = transform.transform.translation.z;
  odom_msg.pose.pose.orientation = transform.transform.rotation;

  // Compute velocity from difference in transforms
  computeVelocity(odom_msg);
  
  // Update state for next iteration
  updateState(odom_msg);

  return odom_msg;
}

void OdometryCore::reset() {
  has_last_transform_ = false;
  RCLCPP_INFO(logger_, "Odometry Core reset");
}

void OdometryCore::updateState(const nav_msgs::msg::Odometry& odom) {
  last_time_ = odom.header.stamp;
  last_position_.setValue(
    odom.pose.pose.position.x,
    odom.pose.pose.position.y,
    odom.pose.pose.position.z
  );
  last_orientation_.setValue(
    odom.pose.pose.orientation.x,
    odom.pose.pose.orientation.y,
    odom.pose.pose.orientation.z,
    odom.pose.pose.orientation.w
  );
  has_last_transform_ = true;
}

void OdometryCore::computeVelocity(nav_msgs::msg::Odometry& odom) const {
  if (has_last_transform_) {
    double dt = computeTimeDifference(odom.header.stamp);
    
    if (dt > 0.0) {
      computeLinearVelocity(odom);
      computeAngularVelocity(odom);
    } else {
      setZeroVelocity(odom);
    }
  } else {
    setZeroVelocity(odom);
  }
}

void OdometryCore::computeLinearVelocity(nav_msgs::msg::Odometry& odom) const {
  double dt = computeTimeDifference(odom.header.stamp);
  
  // Linear velocity
  double dx = odom.pose.pose.position.x - last_position_.x();
  double dy = odom.pose.pose.position.y - last_position_.y();
  double dz = odom.pose.pose.position.z - last_position_.z();

  odom.twist.twist.linear.x = dx / dt;
  odom.twist.twist.linear.y = dy / dt;
  odom.twist.twist.linear.z = dz / dt;
}

void OdometryCore::computeAngularVelocity(nav_msgs::msg::Odometry& odom) const {
  double dt = computeTimeDifference(odom.header.stamp);
  
  // Angular velocity
  tf2::Quaternion q_last(last_orientation_.x(),
                         last_orientation_.y(),
                         last_orientation_.z(),
                         last_orientation_.w());

  tf2::Quaternion q_current(odom.pose.pose.orientation.x,
                            odom.pose.pose.orientation.y,
                            odom.pose.pose.orientation.z,
                            odom.pose.pose.orientation.w);

  // Orientation difference: q_diff = q_last.inverse() * q_current
  tf2::Quaternion q_diff = q_last.inverse() * q_current;

  double roll_diff, pitch_diff, yaw_diff;
  tf2::Matrix3x3(q_diff).getRPY(roll_diff, pitch_diff, yaw_diff);

  // Angular velocity (rad/s)
  odom.twist.twist.angular.x = roll_diff  / dt;
  odom.twist.twist.angular.y = pitch_diff / dt;
  odom.twist.twist.angular.z = yaw_diff   / dt;
}

double OdometryCore::computeTimeDifference(const rclcpp::Time& current_time) const {
  return (current_time - last_time_).seconds();
}

void OdometryCore::setZeroVelocity(nav_msgs::msg::Odometry& odom) const {
  // If dt == 0 or no previous transform, set velocity to zero
  odom.twist.twist.linear.x  = 0.0;
  odom.twist.twist.linear.y  = 0.0;
  odom.twist.twist.linear.z  = 0.0;
  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = 0.0;
  odom.twist.twist.angular.z = 0.0;
}

} 
