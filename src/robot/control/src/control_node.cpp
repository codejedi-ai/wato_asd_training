#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>
#include <optional>
#include "control_core.hpp"

class PurePursuitController : public rclcpp::Node {
public:
    PurePursuitController() : Node("pure_pursuit_controller"), core_(this->get_logger()) {
        // Initialize parameters
        double lookahead_distance = 1.0;  // Lookahead distance
        double goal_tolerance = 0.1;     // Distance to consider the goal reached
        double linear_speed = 0.5;       // Constant forward speed
        
        // Set parameters in core
        core_.setParameters(lookahead_distance, goal_tolerance, linear_speed);
 
        // Subscribers and Publishers
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { 
                current_path_ = msg; 
                RCLCPP_INFO(this->get_logger(), "Received path with %zu waypoints", msg->poses.size());
            });
 
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { 
                robot_odom_ = msg; 
            });
 
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
 
        // Timer for control loop (10 Hz)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), [this]() { controlLoop(); });
            
        RCLCPP_INFO(this->get_logger(), "Pure Pursuit Controller started");
    }
 
private:
    void controlLoop() {
        // Skip control if no path or odometry data is available
        if (!current_path_ || !robot_odom_) {
            return;
        }
        
        // Check if goal is reached
        if (core_.isGoalReached(current_path_, robot_odom_)) {
            // Stop the robot
            geometry_msgs::msg::Twist stop_cmd;
            stop_cmd.linear.x = 0.0;
            stop_cmd.angular.z = 0.0;
            cmd_vel_pub_->publish(stop_cmd);
            
            RCLCPP_INFO(this->get_logger(), "Goal reached! Stopping robot.");
            return;
        }
 
        // Compute velocity command using core
        auto cmd_vel = core_.computeVelocity(current_path_, robot_odom_);
 
        // Publish the velocity command
        cmd_vel_pub_->publish(cmd_vel);
    }
 
    // Subscribers and Publishers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
 
    // Timer
    rclcpp::TimerBase::SharedPtr control_timer_;
 
    // Data
    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr robot_odom_;
    
    // Core functionality
    robot::ControlCore core_;
};
 
// Main function
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuitController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}