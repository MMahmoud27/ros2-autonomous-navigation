#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iterator>
#include <memory>

#include "map_memory_node.hpp"

namespace
{
// Odometry arrives at 10 Hz, so this keeps the last 5 s; a costmap is always much newer than that
constexpr size_t kOdomHistorySize = 50;
// Costmaps arrive at 10 Hz and the map updates at 1 Hz, so this holds about the last second of them
constexpr size_t kPendingCostmaps = 10;
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
  params.max_update_interval = this->declare_parameter("max_update_interval", params.max_update_interval);
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
  pending_costmaps_.push_back(costmap);
  if (pending_costmaps_.size() > kPendingCostmaps) {
    pending_costmaps_.pop_front();
  }
}

void MapMemoryNode::updateMap() {
  // Place the newest costmap that has odometry from both before and after its scan: the obstacles
  // go where the robot was at the moment of the scan. (Pairing a costmap with the latest odometry
  // when it arrives used readings up to 100 ms old, which during turns put far walls ~0.4 m off.)
  for (auto it = pending_costmaps_.rbegin(); it != pending_costmaps_.rend(); ++it) {
    const rclcpp::Time scan_time((*it)->header.stamp);
    const auto after = std::find_if(odom_history_.begin(), odom_history_.end(),
      [&scan_time](const OdomSample& s) { return s.stamp > scan_time; });
    if (after == odom_history_.begin() || after == odom_history_.end()) {
      continue;  // no odometry on one side of this scan (yet)
    }
    const auto before = std::prev(after);
    const double t = scan_time.seconds();
    const robot::Pose2D pose = robot::interpolatePose(
      before->pose, before->stamp.seconds(), after->pose, after->stamp.seconds(), t);
    const double turn_rate = std::max(std::abs(before->turn_rate), std::abs(after->turn_rate));

    if (map_memory_.shouldIntegrate(pose, turn_rate, t)) {
      map_memory_.integrateCostmap(**it, pose, t);
      RCLCPP_DEBUG(this->get_logger(), "Merged costmap taken at (%.2f, %.2f)", pose.x, pose.y);
    }
    // This costmap and every older one have now been dealt with
    pending_costmaps_.erase(pending_costmaps_.begin(), it.base());
    break;
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
