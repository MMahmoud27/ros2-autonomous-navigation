#include "costmap_core.hpp"

#include <algorithm>
#include <cmath>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger, const CostmapParams& params)
: logger_(logger),
  params_(params),
  width_cells_(static_cast<int>(std::lround(params.width_m / params.resolution))),
  height_cells_(static_cast<int>(std::lround(params.height_m / params.resolution)))
{
  // Precompute every cell offset inside the inflation radius together with its cost, so that
  // inflating an obstacle is just a walk over this list. Offset (0, 0) carries max_cost, which
  // is what marks the obstacle cell itself.
  const int radius_cells = static_cast<int>(std::ceil(params_.inflation_radius / params_.resolution));
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const double distance = std::hypot(dx, dy) * params_.resolution;
      if (distance > params_.inflation_radius) {
        continue;
      }
      const auto cost = static_cast<int8_t>(params_.max_cost * (1.0 - distance / params_.inflation_radius));
      if (cost > 0) {
        inflation_stencil_.push_back({dx, dy, cost});
      }
    }
  }
  RCLCPP_INFO(logger_, "Costmap %d x %d cells at %.2f m, inflation radius %.2f m",
    width_cells_, height_cells_, params_.resolution, params_.inflation_radius);
}

nav_msgs::msg::OccupancyGrid CostmapCore::buildCostmap(const sensor_msgs::msg::LaserScan& scan) const
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header = scan.header;
  grid.info.map_load_time = scan.header.stamp;
  grid.info.resolution = static_cast<float>(params_.resolution);
  grid.info.width = width_cells_;
  grid.info.height = height_cells_;
  // The origin is the grid's bottom-left corner; putting it half a grid away puts the lidar at the centre
  const double origin_x = -width_cells_ * params_.resolution / 2.0;
  const double origin_y = -height_cells_ * params_.resolution / 2.0;
  grid.info.origin.position.x = origin_x;
  grid.info.origin.position.y = origin_y;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(static_cast<size_t>(width_cells_) * height_cells_, 0);

  // 1. Convert each valid beam from polar (range, angle) to a grid cell
  std::vector<std::pair<int, int>> hits;
  hits.reserve(scan.ranges.size());
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const double range = scan.ranges[i];
    // Written so NaN fails too; inf and range_max mean "nothing hit"
    if (!(range > scan.range_min && range < scan.range_max)) {
      continue;
    }
    const double angle = scan.angle_min + i * scan.angle_increment;
    const int cx = static_cast<int>(std::floor((range * std::cos(angle) - origin_x) / params_.resolution));
    const int cy = static_cast<int>(std::floor((range * std::sin(angle) - origin_y) / params_.resolution));
    if (cx >= 0 && cx < width_cells_ && cy >= 0 && cy < height_cells_) {
      hits.emplace_back(cx, cy);
    }
  }

  // 2. Stamp the inflation stencil around every hit, keeping the higher cost where zones overlap
  for (const auto& [hx, hy] : hits) {
    for (const auto& offset : inflation_stencil_) {
      const int x = hx + offset.dx;
      const int y = hy + offset.dy;
      if (x < 0 || x >= width_cells_ || y < 0 || y >= height_cells_) {
        continue;
      }
      int8_t& cell = grid.data[static_cast<size_t>(y) * width_cells_ + x];
      cell = std::max(cell, offset.cost);
    }
  }

  return grid;
}

}
