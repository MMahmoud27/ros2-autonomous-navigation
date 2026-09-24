#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include <chrono>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    // Declares the pure pursuit parameters (defaults overridden by params.yaml) and returns their values
    robot::ControlParams loadParams();

    void pathCallback(const nav_msgs::msg::Path::SharedPtr path);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);

    // Timer: computes and publishes the next velocity command
    void controlLoop();

    // Sends one zero command. The simulator keeps executing the last /cmd_vel it received, so stopping
    // has to be explicit; after that the node stays quiet so the Foxglove teleop panel keeps working.
    void stop(const char* reason);

    // The robot turns about its wheel axle, lidar_to_axle_ metres behind the odometry (lidar) frame
    robot::Pose2D axlePose() const;

    robot::ControlCore control_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<robot::Point2D> path_;
    nav_msgs::msg::Odometry::SharedPtr odom_;
    std::chrono::steady_clock::time_point last_odom_received_;
    bool moving_ = false;

    double lidar_to_axle_;
    double odom_timeout_s_;
};

#endif
