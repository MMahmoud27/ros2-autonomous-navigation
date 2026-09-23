#include <functional>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(this->get_logger(), loadParams()) {
  // Sensor-data QoS keeps only the newest scans: a late scan is useless, so never queue old ones
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", rclcpp::SensorDataQoS(),
    std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

robot::CostmapParams CostmapNode::loadParams() {
  robot::CostmapParams params;
  params.resolution = this->declare_parameter("resolution", params.resolution);
  params.width_m = this->declare_parameter("width_m", params.width_m);
  params.height_m = this->declare_parameter("height_m", params.height_m);
  params.inflation_radius = this->declare_parameter("inflation_radius", params.inflation_radius);
  params.max_cost = this->declare_parameter("max_cost", params.max_cost);
  return params;
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  costmap_pub_->publish(costmap_.buildCostmap(*scan));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
