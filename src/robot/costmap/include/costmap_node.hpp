#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();

  private:
    // Declares the costmap parameters (defaults overridden by params.yaml) and returns their values
    robot::CostmapParams loadParams();

    // Turns every incoming scan into a costmap and publishes it
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    robot::CostmapCore costmap_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
};

#endif
