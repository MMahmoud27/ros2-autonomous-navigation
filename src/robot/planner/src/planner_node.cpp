#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(this->get_logger(), loadParams()) {
  goal_tolerance_ = this->declare_parameter("goal_tolerance", 0.5);
  timeout_s_ = this->declare_parameter("timeout_s", 90.0);
  lidar_to_axle_ = this->declare_parameter("lidar_to_axle", 1.3);
  const double replan_period_s = this->declare_parameter("replan_period_s", 0.5);

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(replan_period_s), std::bind(&PlannerNode::timerCallback, this));
}

robot::PlannerParams PlannerNode::loadParams() {
  robot::PlannerParams params;
  params.lethal_cost = this->declare_parameter("lethal_cost", params.lethal_cost);
  params.cost_weight = this->declare_parameter("cost_weight", params.cost_weight);
  params.goal_snap_radius = this->declare_parameter("goal_snap_radius", params.goal_snap_radius);
  return params;
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
  map_ = map;  // the timer replans on it within replan_period_s
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
  odom_ = odom;
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr goal) {
  if (map_ && !goal->header.frame_id.empty() && goal->header.frame_id != map_->header.frame_id) {
    RCLCPP_WARN(this->get_logger(), "Ignoring goal in frame '%s': goals must be in '%s'",
      goal->header.frame_id.c_str(), map_->header.frame_id.c_str());
    return;
  }
  goal_.x = goal->point.x;
  goal_.y = goal->point.y;
  goal_received_time_ = this->now();
  state_ = State::kNavigating;
  last_status_.reset();
  RCLCPP_INFO(this->get_logger(), "New goal (%.2f, %.2f)", goal_.x, goal_.y);
  planAndPublish();
}

void PlannerNode::timerCallback() {
  if (state_ != State::kNavigating) {
    return;
  }
  const auto axle = axlePosition();
  if (axle && std::hypot(goal_.x - axle->x, goal_.y - axle->y) < goal_tolerance_) {
    stopNavigating("Goal reached");
    return;
  }
  if ((this->now() - goal_received_time_).seconds() > timeout_s_) {
    stopNavigating("Timed out before reaching the goal");
    return;
  }
  planAndPublish();
}

void PlannerNode::planAndPublish() {
  const auto axle = axlePosition();
  if (!map_ || !axle) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Waiting for %s before planning", map_ ? "/odom/filtered" : "/map");
    return;
  }

  const auto started = std::chrono::steady_clock::now();
  const auto result = planner_.planPath(*map_, *axle, goal_);
  const double planning_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();

  if (result.status != last_status_) {
    if (result.status == robot::PlanStatus::kOk) {
      double length = 0.0;
      for (size_t i = 1; i < result.path.size(); ++i) {
        length += std::hypot(result.path[i].x - result.path[i - 1].x, result.path[i].y - result.path[i - 1].y);
      }
      RCLCPP_INFO(this->get_logger(), "Path found: %zu points, %.1f m, planned in %.1f ms",
        result.path.size(), length, planning_ms);
    } else {
      RCLCPP_WARN(this->get_logger(), "No path to (%.2f, %.2f): %s",
        goal_.x, goal_.y, robot::toString(result.status));
    }
    last_status_ = result.status;
  }
  // On failure this is an empty path, so the robot stops rather than follow an outdated one
  publishPath(result.path);
}

void PlannerNode::stopNavigating(const std::string& reason) {
  state_ = State::kWaitingForGoal;
  publishPath({});
  RCLCPP_INFO(this->get_logger(), "%s", reason.c_str());
}

void PlannerNode::publishPath(const std::vector<robot::Point2D>& points) {
  nav_msgs::msg::Path path;
  path.header.frame_id = map_ ? map_->header.frame_id : "sim_world";
  path.header.stamp = odom_ ? rclcpp::Time(odom_->header.stamp) : this->now();
  for (const auto& point : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  path_pub_->publish(path);
}

std::optional<robot::Point2D> PlannerNode::axlePosition() const {
  if (!odom_) {
    return std::nullopt;
  }
  const auto& q = odom_->pose.pose.orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  robot::Point2D axle;
  axle.x = odom_->pose.pose.position.x - lidar_to_axle_ * std::cos(yaw);
  axle.y = odom_->pose.pose.position.y - lidar_to_axle_ * std::sin(yaw);
  return axle;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
