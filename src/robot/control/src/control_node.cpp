#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <optional>
 
class PurePursuitController : public rclcpp::Node {
public:
    PurePursuitController() : Node("pure_pursuit_controller") {
        // Initialize parameters
        lookahead_distance_ = 1.0;  // Lookahead distance
        goal_tolerance_ = 0.1;     // Distance to consider the goal reached
        linear_speed_ = 0.5;       // Constant forward speed
 
        // Subscribers and Publishers
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });
 
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });
 
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
 
        // Timer
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), [this]() { controlLoop(); });
    }
 
private:
    void controlLoop() {
        // Skip control if no path or odometry data is available
        if (!current_path_ || !robot_odom_) {
            return;
        }
 
        // Find the lookahead point
        auto lookahead_point = findLookaheadPoint();
        if (!lookahead_point) {
            return;  // No valid lookahead point found
        }
 
        // Compute velocity command
        auto cmd_vel = computeVelocity(*lookahead_point);
 
        // Publish the velocity command
        cmd_vel_pub_->publish(cmd_vel);
    }
 
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint() {
        if (!current_path_ || current_path_->poses.empty()) {
            return std::nullopt;
        }
        
        // Find the closest point on the path to the robot
        int closest_index = 0;
        double min_distance = computeDistance(robot_odom_->pose.pose.position, current_path_->poses[0].pose.position);
        
        for (int i = 1; i < static_cast<int>(current_path_->poses.size()); ++i) {
            double distance = computeDistance(robot_odom_->pose.pose.position, current_path_->poses[i].pose.position);
            if (distance < min_distance) {
                min_distance = distance;
                closest_index = i;
            }
        }
        
        // Search forward from the closest point for the lookahead point
        for (int i = closest_index; i < static_cast<int>(current_path_->poses.size()); ++i) {
            double distance = computeDistance(robot_odom_->pose.pose.position, current_path_->poses[i].pose.position);
            if (distance >= lookahead_distance_) {
                return current_path_->poses[i];
            }
        }
        
        // If no point is far enough, return the last point
        return current_path_->poses.back();
    }
 
    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
        geometry_msgs::msg::Twist cmd_vel;
        
        // Check if goal is reached
        double distance_to_goal = computeDistance(robot_odom_->pose.pose.position, current_path_->poses.back().pose.position);
        if (distance_to_goal <= goal_tolerance_) {
            // Stop the robot
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            return cmd_vel;
        }
        
        // Extract robot yaw
        double robot_yaw = extractYaw(robot_odom_->pose.pose.orientation);
        
        // Compute vector to target
        double dx = target.pose.position.x - robot_odom_->pose.pose.position.x;
        double dy = target.pose.position.y - robot_odom_->pose.pose.position.y;
        
        // Compute angle to target relative to robot's current heading
        double alpha = std::atan2(dy, dx) - robot_yaw;
        
        // Normalize angle to [-π, π]
        while (alpha > M_PI) alpha -= 2.0 * M_PI;
        while (alpha < -M_PI) alpha += 2.0 * M_PI;
        
        // Pure Pursuit steering angle calculation: steering_angle = 2 * sin(alpha) / L
        double steering_angle = 2.0 * std::sin(alpha) / lookahead_distance_;
        
        // Limit angular velocity
        double max_angular_velocity = 1.0; // rad/s
        if (steering_angle > max_angular_velocity) steering_angle = max_angular_velocity;
        if (steering_angle < -max_angular_velocity) steering_angle = -max_angular_velocity;
        
        // Set velocities
        cmd_vel.linear.x = linear_speed_;
        cmd_vel.angular.z = steering_angle;
        
        return cmd_vel;
    }
 
    double computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return std::sqrt(dx * dx + dy * dy);
    }
 
    double extractYaw(const geometry_msgs::msg::Quaternion &quat) {
        double x = quat.x;
        double y = quat.y;
        double z = quat.z;
        double w = quat.w;
        
        return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
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
 
    // Parameters
    double lookahead_distance_;
    double goal_tolerance_;
    double linear_speed_;
};
 
// Main function
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuitController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}