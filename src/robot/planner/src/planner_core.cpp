#include "planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <utility>

namespace robot
{

namespace
{

constexpr double kSqrt2 = 1.4142135623730951;

// The 8 neighbours of a cell: 4 straight moves then 4 diagonals
constexpr int kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
constexpr int kDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

}  // namespace

const char* toString(PlanStatus status)
{
  switch (status) {
    case PlanStatus::kOk: return "ok";
    case PlanStatus::kStartOutsideMap: return "start is outside the map";
    case PlanStatus::kGoalOutsideMap: return "goal is outside the map";
    case PlanStatus::kGoalBlocked: return "goal is inside an obstacle's blocked zone";
    case PlanStatus::kNoPath: return "no route avoids the obstacles";
  }
  return "unknown";
}

PlannerCore::PlannerCore(const rclcpp::Logger& logger, const PlannerParams& params)
: logger_(logger), params_(params) {}

PlanResult PlannerCore::planPath(
  const nav_msgs::msg::OccupancyGrid& map, const Point2D& start, const Point2D& goal) const
{
  PlanResult result;
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  const double res = map.info.resolution;
  const double origin_x = map.info.origin.position.x;
  const double origin_y = map.info.origin.position.y;

  // Cells are numbered id = y * width + x, so every per-cell table below is a flat vector
  auto cellOf = [&](const Point2D& p, int& id) {
    const int x = static_cast<int>(std::floor((p.x - origin_x) / res));
    const int y = static_cast<int>(std::floor((p.y - origin_y) / res));
    if (x < 0 || x >= width || y < 0 || y >= height) {
      return false;
    }
    id = y * width + x;
    return true;
  };
  // Unknown (-1) is treated as free: the robot plans optimistically through unexplored space
  auto costOf = [&](int id) { return std::max<int>(0, map.data[id]); };

  int start_id = 0;
  int goal_id = 0;
  if (!cellOf(start, start_id)) {
    result.status = PlanStatus::kStartOutsideMap;
    return result;
  }
  if (!cellOf(goal, goal_id)) {
    result.status = PlanStatus::kGoalOutsideMap;
    return result;
  }

  // A goal clicked inside the blocked zone moves to the nearest free cell within goal_snap_radius
  if (costOf(goal_id) >= params_.lethal_cost) {
    const int reach = static_cast<int>(std::ceil(params_.goal_snap_radius / res));
    const int gx = goal_id % width;
    const int gy = goal_id / width;
    double best = std::numeric_limits<double>::max();
    int best_id = -1;
    for (int y = std::max(0, gy - reach); y <= std::min(height - 1, gy + reach); ++y) {
      for (int x = std::max(0, gx - reach); x <= std::min(width - 1, gx + reach); ++x) {
        const double d = std::hypot(origin_x + (x + 0.5) * res - goal.x, origin_y + (y + 0.5) * res - goal.y);
        if (d <= params_.goal_snap_radius && d < best && costOf(y * width + x) < params_.lethal_cost) {
          best = d;
          best_id = y * width + x;
        }
      }
    }
    if (best_id < 0) {
      result.status = PlanStatus::kGoalBlocked;
      return result;
    }
    goal_id = best_id;
  }

  // Blocked cells are off limits, except that a start inside the blocked zone may cross cells no
  // costlier than its own: it can drive away from the obstacle, but never closer to it
  const int start_cost = costOf(start_id);
  auto passable = [&](int id) {
    const int cost = costOf(id);
    return cost < params_.lethal_cost || cost <= start_cost;
  };

  // Octile distance: the exact cost of the shortest 8-connected route with no obstacles. It never
  // overestimates, so A* still returns the cheapest path.
  const int goal_x = goal_id % width;
  const int goal_y = goal_id / width;
  auto heuristic = [&](int id) {
    const int dx = std::abs(id % width - goal_x);
    const int dy = std::abs(id / width - goal_y);
    return (std::max(dx, dy) - std::min(dx, dy) + kSqrt2 * std::min(dx, dy)) * res;
  };

  const size_t cell_count = static_cast<size_t>(width) * height;
  std::vector<double> g_score(cell_count, std::numeric_limits<double>::infinity());
  std::vector<int> came_from(cell_count, -1);
  std::vector<bool> closed(cell_count, false);

  // Open set: min-heap on f = g + h
  using Entry = std::pair<double, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  g_score[start_id] = 0.0;
  open.emplace(heuristic(start_id), start_id);

  while (!open.empty()) {
    const int current = open.top().second;
    open.pop();
    if (closed[current]) {
      continue;  // stale queue entry: this cell was already expanded with a lower cost
    }
    if (current == goal_id) {
      break;
    }
    closed[current] = true;

    const int cx = current % width;
    const int cy = current / width;
    for (int k = 0; k < 8; ++k) {
      const int nx = cx + kDx[k];
      const int ny = cy + kDy[k];
      if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
        continue;
      }
      const int neighbour = ny * width + nx;
      if (closed[neighbour] || !passable(neighbour)) {
        continue;
      }
      const bool diagonal = kDx[k] != 0 && kDy[k] != 0;
      // No squeezing diagonally past a blocked corner: both straight cells beside the move must be passable
      if (diagonal && (!passable(cy * width + nx) || !passable(ny * width + cx))) {
        continue;
      }
      const double step = (diagonal ? kSqrt2 : 1.0) * res;
      const double tentative = g_score[current] + step * (1.0 + params_.cost_weight * costOf(neighbour) / 100.0);
      if (tentative < g_score[neighbour]) {
        g_score[neighbour] = tentative;
        came_from[neighbour] = current;
        open.emplace(tentative + heuristic(neighbour), neighbour);
      }
    }
  }

  if (goal_id != start_id && came_from[goal_id] < 0) {
    result.status = PlanStatus::kNoPath;
    return result;
  }

  // Walk back from the goal to the start, then reverse into start -> goal order
  for (int id = goal_id; id != -1; id = came_from[id]) {
    result.path.push_back({origin_x + (id % width + 0.5) * res, origin_y + (id / width + 0.5) * res});
  }
  std::reverse(result.path.begin(), result.path.end());
  result.status = PlanStatus::kOk;
  return result;
}

}
