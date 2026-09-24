#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    // One odometry reading, kept briefly so each costmap can be placed at the pose of its scan time
    struct OdomSample {
      rclcpp::Time stamp;
      robot::Pose2D pose;
      double turn_rate = 0.0;
    };

    // Declares the map parameters (defaults overridden by params.yaml) and returns their values
    robot::MapMemoryParams loadParams();

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);

    // Timer: merges the latest costmap when it's due, then republishes the map
    void updateMap();

    robot::MapMemoryCore map_memory_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::deque<OdomSample> odom_history_;
    // Costmaps not merged yet, oldest first. A costmap can only be placed once odometry from after its
    // scan has arrived, so the pose at the scan time can be interpolated.
    std::deque<nav_msgs::msg::OccupancyGrid::SharedPtr> pending_costmaps_;
};

#endif
