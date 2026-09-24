#include "control_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger, const ControlParams& params)
: logger_(logger), params_(params) {}

std::optional<Point2D> ControlCore::findLookaheadPoint(
  const std::vector<Point2D>& path, const Point2D& position) const
{
  if (path.empty()) {
    return std::nullopt;
  }
  // Start from the path point nearest the robot, so points already driven past are skipped
  size_t closest = 0;
  double closest_distance = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path.size(); ++i) {
    const double d = std::hypot(path[i].x - position.x, path[i].y - position.y);
    if (d < closest_distance) {
      closest_distance = d;
      closest = i;
    }
  }
  for (size_t i = closest; i < path.size(); ++i) {
    if (std::hypot(path[i].x - position.x, path[i].y - position.y) >= params_.lookahead_distance) {
      return path[i];
    }
  }
  return path.back();
}

VelocityCommand ControlCore::computeCommand(const std::vector<Point2D>& path, const Pose2D& pose) const
{
  VelocityCommand cmd;  // zero: stand still
  if (path.empty()) {
    return cmd;
  }
  const double to_goal = std::hypot(path.back().x - pose.x, path.back().y - pose.y);
  if (to_goal < params_.goal_tolerance) {
    return cmd;
  }

  // Lookahead point in the robot's own frame: x forward, y to the left
  const Point2D target = *findLookaheadPoint(path, {pose.x, pose.y});
  const double dx = target.x - pose.x;
  const double dy = target.y - pose.y;
  const double forward = std::cos(pose.yaw) * dx + std::sin(pose.yaw) * dy;
  const double left = -std::sin(pose.yaw) * dx + std::cos(pose.yaw) * dy;
  const double distance_sq = forward * forward + left * left;
  if (distance_sq < 1e-9) {
    return cmd;
  }

  // Target well off to the side or behind: an arc there would be huge, so turn on the spot first
  const double heading_error = std::atan2(left, forward);
  if (std::abs(heading_error) > params_.rotate_in_place_angle) {
    cmd.angular = std::copysign(params_.max_angular_speed, heading_error);
    return cmd;
  }

  // Pure pursuit: the circular arc through the robot and the target has curvature 2y / L^2
  const double curvature = 2.0 * left / distance_sq;

  // Ease off approaching the goal, but keep enough speed to actually arrive
  double speed = params_.linear_speed *
    std::clamp(to_goal / params_.slowdown_distance, params_.min_speed_ratio, 1.0);
  // If the turn rate would exceed the limit, slow down rather than widen the arc
  if (std::abs(speed * curvature) > params_.max_angular_speed) {
    speed = params_.max_angular_speed / std::abs(curvature);
  }
  cmd.linear = speed;
  cmd.angular = speed * curvature;
  return cmd;
}

}
