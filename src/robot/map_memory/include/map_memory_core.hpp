#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

// Tunable map memory settings, loaded from params.yaml by the node
struct MapMemoryParams {
  std::string frame_id = "sim_world";  // world frame the map is expressed in
  double resolution = 0.1;             // metres per cell
  double width_m = 40.0;               // map size; the arena walls are at +-15 m
  double height_m = 40.0;
  double origin_x = -20.0;             // world position of the map's bottom-left corner
  double origin_y = -20.0;
  double update_distance = 1.5;        // merge a new costmap after moving this far (m)...
  double max_update_interval = 2.0;    // ...or after this long without a merge (s)
  double max_turn_rate = 1.0;          // never merge while turning faster than this (rad/s)
};

// Position and heading of a frame, expressed in the map frame
struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

// Heading (rotation about z) of a quaternion, in radians
double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q);

// The pose at time t, blended linearly between pose a (at time ta) and pose b (at time tb), with the
// heading turning the short way round. Used to place a scan at the exact moment it was taken.
Pose2D interpolatePose(const Pose2D& a, double ta, const Pose2D& b, double tb, double t);

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger, const MapMemoryParams& params = MapMemoryParams());

    // Whether a costmap taken at `pose` and time `time_s` should be merged. Never while turning faster
    // than max_turn_rate (the scan and its pose can't be matched exactly, so a turn smears obstacles);
    // otherwise always the first time, then once the robot has moved update_distance or
    // max_update_interval has passed. The time rule matters at startup: the first scans can arrive
    // before the simulator has loaded the world, and a robot that isn't moving must still catch up.
    bool shouldIntegrate(const Pose2D& pose, double turn_rate, double time_s) const;

    // Merges a costmap into the map. `pose` is where the costmap's frame was in the map frame when
    // its scan was taken, at `time_s`. Each map cell under the costmap keeps the higher of its old and
    // new cost; unknown (-1) costmap cells leave the map unchanged.
    void integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap, const Pose2D& pose, double time_s);

    // The accumulated map; cells nobody has seen yet are -1 (unknown)
    const nav_msgs::msg::OccupancyGrid& map() const;

  private:
    rclcpp::Logger logger_;
    MapMemoryParams params_;
    nav_msgs::msg::OccupancyGrid map_;
    std::optional<Pose2D> last_integration_pose_;
    double last_integration_time_s_ = 0.0;
};

}

#endif
