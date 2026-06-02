#include "diff_drive_controller.hpp"

namespace robot_control {

DiffDriveController::DiffDriveController(const rclcpp::NodeOptions &options)
    : Node("diff_drive_controller", options) {
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&DiffDriveController::cmd_vel_callback, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/wheel_odom", 10);

    RCLCPP_INFO(this->get_logger(), "DiffDriveController initialized (stub)");
}

void DiffDriveController::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr /*msg*/) {
    // Stub: Gazebo diff_drive plugin handles actual control in simulation
    // Real hardware control will be implemented here
}

void DiffDriveController::publish_odometry() {
    // Stub: Odometry published by Gazebo plugin in simulation
    // Real odometry computation will be implemented here
}

}  // namespace robot_control

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robot_control::DiffDriveController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
