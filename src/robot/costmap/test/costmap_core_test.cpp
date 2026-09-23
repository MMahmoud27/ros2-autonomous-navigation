#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "costmap_core.hpp"

// With the default params the grid is 40 x 40 m at 0.1 m per cell, centred on the lidar.
// A point (x, y) in metres therefore lands in cell (floor((x + 20) / 0.1), floor((y + 20) / 0.1)).
// Test points sit in the middle of a cell so floating-point rounding can't move them across an edge.

namespace
{

sensor_msgs::msg::LaserScan makeScan(double angle_min, double angle_increment, const std::vector<float>& ranges)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "robot/chassis/lidar";
  scan.angle_min = angle_min;
  scan.angle_increment = angle_increment;
  scan.angle_max = angle_min + angle_increment * std::max<int>(0, static_cast<int>(ranges.size()) - 1);
  scan.range_min = 0.08;
  scan.range_max = 20.0;
  scan.ranges = ranges;
  return scan;
}

// A scan with one beam per point, each beam hitting that point (metres, lidar frame)
sensor_msgs::msg::LaserScan scanHitting(const std::vector<std::pair<double, double>>& points)
{
  const double first = std::atan2(points[0].second, points[0].first);
  const double step = points.size() > 1 ? std::atan2(points[1].second, points[1].first) - first : 0.0;
  std::vector<float> ranges;
  for (const auto& [x, y] : points) {
    ranges.push_back(static_cast<float>(std::hypot(x, y)));
  }
  return makeScan(first, step, ranges);
}

int costAt(const nav_msgs::msg::OccupancyGrid& grid, int cx, int cy)
{
  return grid.data.at(cy * grid.info.width + cx);
}

bool allFree(const nav_msgs::msg::OccupancyGrid& grid)
{
  return std::all_of(grid.data.begin(), grid.data.end(), [](int8_t c) { return c == 0; });
}

}  // namespace

class CostmapCoreTest : public ::testing::Test
{
protected:
  robot::CostmapCore core{rclcpp::get_logger("costmap_core_test")};
};

TEST_F(CostmapCoreTest, GridIsCentredOnLidarAndKeepsScanFrame)
{
  const auto grid = core.buildCostmap(makeScan(0.0, 0.1, {}));

  EXPECT_EQ(grid.header.frame_id, "robot/chassis/lidar");
  EXPECT_EQ(grid.info.width, 400u);
  EXPECT_EQ(grid.info.height, 400u);
  EXPECT_FLOAT_EQ(grid.info.resolution, 0.1f);
  EXPECT_DOUBLE_EQ(grid.info.origin.position.x, -20.0);
  EXPECT_DOUBLE_EQ(grid.info.origin.position.y, -20.0);
  EXPECT_DOUBLE_EQ(grid.info.origin.orientation.w, 1.0);
  EXPECT_EQ(grid.data.size(), 160000u);
}

TEST_F(CostmapCoreTest, ScanWithNoHitsGivesAllFreeCells)
{
  const float inf = std::numeric_limits<float>::infinity();
  const auto grid = core.buildCostmap(makeScan(-1.0, 0.5, {inf, inf, inf, inf, inf}));

  ASSERT_EQ(grid.data.size(), 160000u);
  EXPECT_TRUE(allFree(grid));
}

TEST_F(CostmapCoreTest, HitIsMarkedAsObstacleInTheRightCell)
{
  // (3.05, 4.05) m -> cell (floor(23.05 / 0.1), floor(24.05 / 0.1)) = (230, 240)
  const auto grid = core.buildCostmap(scanHitting({{3.05, 4.05}}));

  EXPECT_EQ(costAt(grid, 230, 240), 100);
}

TEST_F(CostmapCoreTest, InvalidRangesAreIgnored)
{
  const float inf = std::numeric_limits<float>::infinity();
  const float nan = std::numeric_limits<float>::quiet_NaN();
  // inf = no return, NaN = bad reading, 0.05 < range_min, 20.0 == range_max (max-range "miss")
  const auto grid = core.buildCostmap(makeScan(0.3, 0.2, {inf, nan, 0.05f, 20.0f}));

  ASSERT_EQ(grid.data.size(), 160000u);
  EXPECT_TRUE(allFree(grid));
}

TEST_F(CostmapCoreTest, InflationFallsOffLinearlyWithDistance)
{
  // Obstacle at cell (230, 240); default inflation radius 1.6 m, cost = 100 * (1 - d / 1.6)
  const auto grid = core.buildCostmap(scanHitting({{3.05, 4.05}}));

  EXPECT_EQ(costAt(grid, 235, 240), 68);  // 5 cells right: d = 0.5 m -> 68.75 -> 68
  EXPECT_EQ(costAt(grid, 233, 244), 68);  // (3, 4) cells diagonal: d = 0.5 m -> 68
  EXPECT_EQ(costAt(grid, 230, 250), 37);  // 10 cells up: d = 1.0 m -> 37.5 -> 37
  EXPECT_EQ(costAt(grid, 230, 256), 0);   // 16 cells: d = 1.6 m, on the radius -> 0
  EXPECT_EQ(costAt(grid, 250, 240), 0);   // 20 cells: outside the radius
}

TEST_F(CostmapCoreTest, OverlappingInflationKeepsTheHighestCostNotTheSum)
{
  // Obstacles at cells (230, 240) and (240, 240); cell (235, 240) is 0.5 m from both
  const auto grid = core.buildCostmap(scanHitting({{3.05, 4.05}, {4.05, 4.05}}));

  EXPECT_EQ(costAt(grid, 230, 240), 100);
  EXPECT_EQ(costAt(grid, 240, 240), 100);
  EXPECT_EQ(costAt(grid, 235, 240), 68);  // not 136, and not clipped to 100
}

TEST(CostmapCoreParamsTest, HitOutsideTheGridIsIgnored)
{
  robot::CostmapParams params;
  params.width_m = 4.0;   // grid spans -2..2 m around the lidar
  params.height_m = 4.0;
  robot::CostmapCore core(rclcpp::get_logger("costmap_core_test"), params);

  const auto grid = core.buildCostmap(scanHitting({{5.05, 0.05}}));  // 5 m ahead: off the grid

  ASSERT_EQ(grid.data.size(), 1600u);
  EXPECT_TRUE(allFree(grid));
}
