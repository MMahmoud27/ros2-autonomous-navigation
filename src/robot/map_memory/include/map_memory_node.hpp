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
    // One odometry reading, kept briefly so each costmap can be paired with the pose at its scan time
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
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
    OdomSample latest_costmap_odom_;  // the odometry sample closest in time to latest_costmap_
};

#endif
