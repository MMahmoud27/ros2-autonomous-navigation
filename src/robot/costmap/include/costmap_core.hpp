#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <cstdint>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot
{

// Tunable costmap settings, loaded from params.yaml by the node
struct CostmapParams {
  double resolution = 0.1;        // metres per cell
  double width_m = 40.0;          // grid size in metres, centred on the lidar
  double height_m = 40.0;
  double inflation_radius = 1.6;  // metres; cost falls linearly to 0 at this distance
  int max_cost = 100;             // cost of a cell that contains an obstacle
};

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger, const CostmapParams& params = CostmapParams());

    // Builds an inflated costmap from one laser scan. The grid is centred on the lidar and
    // expressed in the scan's frame, so the scan header is copied into the result.
    nav_msgs::msg::OccupancyGrid buildCostmap(const sensor_msgs::msg::LaserScan& scan) const;

  private:
    // One entry of the precomputed inflation stencil: a cell offset and the cost at that distance
    struct InflationOffset {
      int dx;
      int dy;
      int8_t cost;
    };

    rclcpp::Logger logger_;
    CostmapParams params_;
    int width_cells_;
    int height_cells_;
    std::vector<InflationOffset> inflation_stencil_;
};

}

#endif
