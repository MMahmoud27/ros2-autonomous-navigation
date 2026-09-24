#include <cmath>
#include <functional>
#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(this->get_logger(), loadParams()) {
  lidar_to_axle_ = this->declare_parameter("lidar_to_axle", 1.3);
  odom_timeout_s_ = this->declare_parameter("odom_timeout_s", 0.5);
  const double rate_hz = this->declare_parameter("control_rate_hz", 10.0);

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / rate_hz), std::bind(&ControlNode::controlLoop, this));
}

robot::ControlParams ControlNode::loadParams() {
  robot::ControlParams params;
  params.lookahead_distance = this->declare_parameter("lookahead_distance", params.lookahead_distance);
  params.linear_speed = this->declare_parameter("linear_speed", params.linear_speed);
  params.max_angular_speed = this->declare_parameter("max_angular_speed", params.max_angular_speed);
  params.goal_tolerance = this->declare_parameter("goal_tolerance", params.goal_tolerance);
  params.rotate_in_place_angle = this->declare_parameter("rotate_in_place_angle", params.rotate_in_place_angle);
  params.slowdown_distance = this->declare_parameter("slowdown_distance", params.slowdown_distance);
  params.min_speed_ratio = this->declare_parameter("min_speed_ratio", params.min_speed_ratio);
  return params;
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr path) {
  if (odom_ && !path->poses.empty() && path->header.frame_id != odom_->header.frame_id) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Ignoring path in frame '%s': odometry is in '%s'",
      path->header.frame_id.c_str(), odom_->header.frame_id.c_str());
    return;
  }
  path_.clear();
  for (const auto& pose : path->poses) {
    path_.push_back({pose.pose.position.x, pose.pose.position.y});
  }
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
  odom_ = odom;
  last_odom_received_ = std::chrono::steady_clock::now();
}

void ControlNode::controlLoop() {
  if (!odom_) {
    return;
  }
  const double odom_age_s =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - last_odom_received_).count();
  if (odom_age_s > odom_timeout_s_) {
    if (moving_) {
      stop("Odometry stopped arriving");
    }
    return;
  }

  const auto cmd = control_.computeCommand(path_, axlePose());
  if (cmd.linear == 0.0 && cmd.angular == 0.0) {
    if (moving_) {
      stop(path_.empty() ? "Path cleared" : "Reached the end of the path");
    }
    return;
  }

  if (!moving_) {
    RCLCPP_INFO(this->get_logger(), "Following path (%zu points)", path_.size());
  }
  geometry_msgs::msg::Twist twist;
  twist.linear.x = cmd.linear;
  twist.angular.z = cmd.angular;
  cmd_vel_pub_->publish(twist);
  moving_ = true;
}

void ControlNode::stop(const char* reason) {
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
  moving_ = false;
  RCLCPP_INFO(this->get_logger(), "%s: stopping", reason);
}

robot::Pose2D ControlNode::axlePose() const {
  const auto& q = odom_->pose.pose.orientation;
  robot::Pose2D pose;
  pose.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  pose.x = odom_->pose.pose.position.x - lidar_to_axle_ * std::cos(pose.yaw);
  pose.y = odom_->pose.pose.position.y - lidar_to_axle_ * std::sin(pose.yaw);
  return pose;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
