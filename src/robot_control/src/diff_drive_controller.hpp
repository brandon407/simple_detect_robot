#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace robot_control {

/// Differential drive controller node (Phase 1 stub).
/// In simulation, Gazebo diff_drive plugin handles this.
/// This node serves as a bridge for real hardware control.
class DiffDriveController : public rclcpp::Node {
public:
    explicit DiffDriveController(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~DiffDriveController() override = default;

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void publish_odometry();

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

}  // namespace robot_control
