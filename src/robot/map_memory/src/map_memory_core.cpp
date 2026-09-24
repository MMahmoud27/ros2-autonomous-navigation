#include "map_memory_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace robot
{

double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
{
  // Standard quaternion -> yaw (z-axis rotation) conversion
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger, const MapMemoryParams& params)
  : logger_(logger), params_(params)
{
  map_.header.frame_id = params_.frame_id;
  map_.info.resolution = static_cast<float>(params_.resolution);
  map_.info.width = static_cast<uint32_t>(std::lround(params_.width_m / params_.resolution));
  map_.info.height = static_cast<uint32_t>(std::lround(params_.height_m / params_.resolution));
  map_.info.origin.position.x = params_.origin_x;
  map_.info.origin.position.y = params_.origin_y;
  map_.info.origin.orientation.w = 1.0;
  map_.data.assign(static_cast<size_t>(map_.info.width) * map_.info.height, -1);

  RCLCPP_INFO(logger_, "Map %u x %u cells at %.2f m in '%s'",
    map_.info.width, map_.info.height, params_.resolution, params_.frame_id.c_str());
}

bool MapMemoryCore::shouldIntegrate(const Pose2D& pose, double turn_rate, double time_s) const
{
  if (std::abs(turn_rate) > params_.max_turn_rate) {
    return false;
  }
  if (!last_integration_pose_) {
    return true;
  }
  const double moved = std::hypot(pose.x - last_integration_pose_->x, pose.y - last_integration_pose_->y);
  return moved >= params_.update_distance || time_s - last_integration_time_s_ >= params_.max_update_interval;
}

void MapMemoryCore::integrateCostmap(
  const nav_msgs::msg::OccupancyGrid& costmap, const Pose2D& pose, double time_s)
{
  const double cm_res = costmap.info.resolution;
  const int cm_width = static_cast<int>(costmap.info.width);
  const int cm_height = static_cast<int>(costmap.info.height);
  const double cm_origin_x = costmap.info.origin.position.x;
  const double cm_origin_y = costmap.info.origin.position.y;
  const double cos_yaw = std::cos(pose.yaw);
  const double sin_yaw = std::sin(pose.yaw);

  // World-frame bounding box of the (rotated) costmap, so only map cells it can cover are visited
  const std::array<std::pair<double, double>, 4> corners = {{
    {cm_origin_x, cm_origin_y},
    {cm_origin_x + cm_width * cm_res, cm_origin_y},
    {cm_origin_x, cm_origin_y + cm_height * cm_res},
    {cm_origin_x + cm_width * cm_res, cm_origin_y + cm_height * cm_res},
  }};
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  for (const auto& [cx, cy] : corners) {
    const double wx = pose.x + cos_yaw * cx - sin_yaw * cy;
    const double wy = pose.y + sin_yaw * cx + cos_yaw * cy;
    min_x = std::min(min_x, wx);
    max_x = std::max(max_x, wx);
    min_y = std::min(min_y, wy);
    max_y = std::max(max_y, wy);
  }

  const int map_width = static_cast<int>(map_.info.width);
  const int map_height = static_cast<int>(map_.info.height);
  const double res = params_.resolution;
  const int x_begin = std::max(0, static_cast<int>(std::floor((min_x - params_.origin_x) / res)));
  const int x_end = std::min(map_width - 1, static_cast<int>(std::floor((max_x - params_.origin_x) / res)));
  const int y_begin = std::max(0, static_cast<int>(std::floor((min_y - params_.origin_y) / res)));
  const int y_end = std::min(map_height - 1, static_cast<int>(std::floor((max_y - params_.origin_y) / res)));

  // For every map cell, look up the costmap cell it falls in (rather than pushing costmap cells
  // into the map), so a rotated costmap can't leave holes between cells
  for (int my = y_begin; my <= y_end; ++my) {
    for (int mx = x_begin; mx <= x_end; ++mx) {
      // Centre of this map cell in the world, then in the costmap's frame (inverse rotation)
      const double dx = params_.origin_x + (mx + 0.5) * res - pose.x;
      const double dy = params_.origin_y + (my + 0.5) * res - pose.y;
      const double local_x = cos_yaw * dx + sin_yaw * dy;
      const double local_y = -sin_yaw * dx + cos_yaw * dy;

      const int cx = static_cast<int>(std::floor((local_x - cm_origin_x) / cm_res));
      const int cy = static_cast<int>(std::floor((local_y - cm_origin_y) / cm_res));
      if (cx < 0 || cx >= cm_width || cy < 0 || cy >= cm_height) {
        continue;
      }
      const int8_t cost = costmap.data[static_cast<size_t>(cy) * cm_width + cx];
      if (cost < 0) {
        continue;  // the costmap doesn't know this cell: keep what the map had
      }
      int8_t& cell = map_.data[static_cast<size_t>(my) * map_width + mx];
      cell = std::max(cell, cost);  // unknown (-1) is below every real cost, so it's replaced
    }
  }

  last_integration_pose_ = pose;
  last_integration_time_s_ = time_s;
}

const nav_msgs::msg::OccupancyGrid& MapMemoryCore::map() const
{
  return map_;
}

}
