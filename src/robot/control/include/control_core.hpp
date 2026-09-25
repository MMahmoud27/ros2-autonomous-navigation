#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <optional>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace robot
{

// Tunable controller settings, loaded from params.yaml by the node
struct ControlParams {
  double lookahead_distance = 1.8;     // steer toward the path point this far ahead (m)
  double linear_speed = 1.5;           // cruising speed (m/s); at 3 m/s the robot pitches ~16 deg and the lidar sees the floor
  double max_angular_speed = 1.5;      // turn rate limit (rad/s)
  double goal_tolerance = 0.3;         // stop when the path's end is this close (m)
  double rotate_in_place_angle = 1.0;  // turn on the spot when the target is more than this off heading (rad)
  double slowdown_distance = 2.5;      // start slowing down this far from the goal (m)
  double min_speed_ratio = 0.25;       // never slow below this fraction of linear_speed while driving
};

struct Point2D {
  double x = 0.0;
  double y = 0.0;
};

// Position of the point being controlled (the wheel axle) and its heading
struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct VelocityCommand {
  double linear = 0.0;   // m/s, forward
  double angular = 0.0;  // rad/s, counter-clockwise
};

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit ControlCore(const rclcpp::Logger& logger, const ControlParams& params = ControlParams());

    // Pure pursuit: the velocity that drives the axle along `path` (map frame, start -> goal).
    // Zero when the path is empty or its end is within goal_tolerance.
    VelocityCommand computeCommand(const std::vector<Point2D>& path, const Pose2D& pose) const;

    // The first path point at least lookahead_distance from `position`, searching forward from the
    // path point closest to the robot (so points already passed are never chosen). If the rest of
    // the path is closer than that, the path's last point. Empty path -> nullopt.
    std::optional<Point2D> findLookaheadPoint(const std::vector<Point2D>& path, const Point2D& position) const;

  private:
    rclcpp::Logger logger_;
    ControlParams params_;
};

}

#endif
