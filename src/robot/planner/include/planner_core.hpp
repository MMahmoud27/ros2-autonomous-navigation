#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

// Tunable planner settings, loaded from params.yaml by the node
struct PlannerParams {
  int lethal_cost = 34;           // cells at or above this cost are blocked (1.65 m from an obstacle)
  int goal_max_cost = 20;         // goals must cost less than this (2.0 m clear), so the robot parks with room to turn
  double cost_weight = 3.0;       // how strongly paths avoid high-cost cells (0 = shortest path only)
  double goal_snap_radius = 2.5;  // a goal costing too much moves to the nearest good cell within this (m)
};

struct Point2D {
  double x = 0.0;
  double y = 0.0;
};

enum class PlanStatus {
  kOk,
  kStartOutsideMap,
  kGoalOutsideMap,
  kGoalBlocked,  // goal is too close to an obstacle, with no better cell within goal_snap_radius
  kNoPath,       // no route between start and goal avoids the blocked cells
};

const char* toString(PlanStatus status);

struct PlanResult {
  PlanStatus status = PlanStatus::kNoPath;
  std::vector<Point2D> path;  // cell centres from start to goal, in the map frame
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger, const PlannerParams& params = PlannerParams());

    // A* on an 8-connected grid from `start` to `goal` (metres, map frame).
    // Moving into a cell costs its step length (1 or sqrt(2) cells) times (1 + cost_weight * cost / 100).
    // Unknown cells (-1) count as free. Cells at or above lethal_cost are blocked, except that a start
    // inside the blocked zone may move through cells no costlier than its own, so it can drive out.
    PlanResult planPath(const nav_msgs::msg::OccupancyGrid& map, const Point2D& start, const Point2D& goal) const;

  private:
    rclcpp::Logger logger_;
    PlannerParams params_;
};

}

#endif
