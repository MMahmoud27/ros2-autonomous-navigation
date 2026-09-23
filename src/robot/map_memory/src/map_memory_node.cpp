#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "map_memory_node.hpp"

namespace
{
// Odometry arrives at 10 Hz, so this keeps the last 5 s; a costmap is always much newer than that
constexpr size_t kOdomHistorySize = 50;
}  // namespace

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(this->get_logger(), loadParams()) {
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // The map is republished every tick, even before the first costmap, so late subscribers
  // (the planner, Foxglove) always get it
  const double period_s = this->declare_parameter("publish_period_s", 1.0);
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(period_s), std::bind(&MapMemoryNode::updateMap, this));
}

robot::MapMemoryParams MapMemoryNode::loadParams() {
  robot::MapMemoryParams params;
  params.frame_id = this->declare_parameter("frame_id", params.frame_id);
  params.resolution = this->declare_parameter("resolution", params.resolution);
  params.width_m = this->declare_parameter("width_m", params.width_m);
  params.height_m = this->declare_parameter("height_m", params.height_m);
  params.origin_x = this->declare_parameter("origin_x", params.origin_x);
  params.origin_y = this->declare_parameter("origin_y", params.origin_y);
  params.update_distance = this->declare_parameter("update_distance", params.update_distance);
  params.max_turn_rate = this->declare_parameter("max_turn_rate", params.max_turn_rate);
  return params;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
  OdomSample sample;
  sample.stamp = rclcpp::Time(odom->header.stamp);
  sample.pose.x = odom->pose.pose.position.x;
  sample.pose.y = odom->pose.pose.position.y;
  sample.pose.yaw = robot::yawFromQuaternion(odom->pose.pose.orientation);
  sample.turn_rate = odom->twist.twist.angular.z;

  odom_history_.push_back(sample);
  if (odom_history_.size() > kOdomHistorySize) {
    odom_history_.pop_front();
  }
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap) {
  if (odom_history_.empty()) {
    return;  // no pose yet to place this costmap with
  }
  // The costmap carries its scan's timestamp; use the odometry reading taken closest to it, so the
  // obstacles are placed where the robot was when it saw them, not where it is now
  const rclcpp::Time scan_time(costmap->header.stamp);
  const auto closest = std::min_element(odom_history_.begin(), odom_history_.end(),
    [&scan_time](const OdomSample& a, const OdomSample& b) {
      return std::abs((a.stamp - scan_time).seconds()) < std::abs((b.stamp - scan_time).seconds());
    });

  latest_costmap_ = costmap;
  latest_costmap_odom_ = *closest;
}

void MapMemoryNode::updateMap() {
  if (latest_costmap_ &&
      map_memory_.shouldIntegrate(latest_costmap_odom_.pose, latest_costmap_odom_.turn_rate)) {
    map_memory_.integrateCostmap(*latest_costmap_, latest_costmap_odom_.pose);
    RCLCPP_INFO(this->get_logger(), "Merged costmap taken at (%.2f, %.2f)",
      latest_costmap_odom_.pose.x, latest_costmap_odom_.pose.y);
    latest_costmap_.reset();
  }

  auto map = map_memory_.map();
  // Stamp with the simulator clock like every other message, once odometry has given us a time
  map.header.stamp = odom_history_.empty() ? this->now() : odom_history_.back().stamp;
  map.info.map_load_time = map.header.stamp;
  map_pub_->publish(map);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
