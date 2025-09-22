#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "control_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <cmath>
#include <optional>

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    robot::ControlCore control_;
    nav_msgs::msg::Odometry::SharedPtr odom_;
    nav_msgs::msg::Path::SharedPtr cur_path_;
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void controlLoop();

    std::optional<geometry_msgs::msg::PoseStamped> findLookahead();
    geometry_msgs::msg::Twist computeVel(const geometry_msgs::msg::PoseStamped& tgt);
    double computeDis(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b);
    bool isNearObstacle(double x, double y, double threshold = 0.5);

    static constexpr double lookahead_distance_ = 1.0;
    static constexpr double lin_kp_ = 0.8;
    static constexpr double ang_kp_ = 0.8;
    static constexpr double max_lin_vel_ = 1;
    static constexpr int dt_ = 100;
    static constexpr double goal_tolerance_ = 0.3;
};

#endif
