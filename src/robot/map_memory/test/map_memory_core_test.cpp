#include <algorithm>
#include <cmath>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "map_memory_core.hpp"

// With the default params the map is 40 x 40 m at 0.1 m per cell, bottom-left corner at (-20, -20)
// in sim_world, so map cell (mx, my) has its centre at (-20 + (mx + 0.5) * 0.1, -20 + (my + 0.5) * 0.1).
//
// The test costmap is 10 x 10 cells at 0.1 m with its corner at (-0.5, -0.5) in the costmap frame,
// so costmap cell (7, 5) is centred on (0.25, 0.05) in that frame.

namespace
{

nav_msgs::msg::OccupancyGrid makeCostmap(int8_t fill = 0)
{
  nav_msgs::msg::OccupancyGrid costmap;
  costmap.header.frame_id = "robot/chassis/lidar";
  costmap.info.resolution = 0.1f;
  costmap.info.width = 10;
  costmap.info.height = 10;
  costmap.info.origin.position.x = -0.5;
  costmap.info.origin.position.y = -0.5;
  costmap.info.origin.orientation.w = 1.0;
  costmap.data.assign(100, fill);
  return costmap;
}

void setCost(nav_msgs::msg::OccupancyGrid& grid, int cx, int cy, int8_t cost)
{
  grid.data.at(cy * grid.info.width + cx) = cost;
}

int costAt(const nav_msgs::msg::OccupancyGrid& grid, int cx, int cy)
{
  return grid.data.at(cy * grid.info.width + cx);
}

geometry_msgs::msg::Quaternion yawQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

}  // namespace

class MapMemoryCoreTest : public ::testing::Test
{
protected:
  robot::MapMemoryCore memory{rclcpp::get_logger("map_memory_core_test")};
};

TEST_F(MapMemoryCoreTest, MapStartsUnknownWithConfiguredGeometry)
{
  const auto& map = memory.map();

  EXPECT_EQ(map.header.frame_id, "sim_world");
  EXPECT_EQ(map.info.width, 400u);
  EXPECT_EQ(map.info.height, 400u);
  EXPECT_FLOAT_EQ(map.info.resolution, 0.1f);
  EXPECT_DOUBLE_EQ(map.info.origin.position.x, -20.0);
  EXPECT_DOUBLE_EQ(map.info.origin.position.y, -20.0);
  ASSERT_EQ(map.data.size(), 160000u);
  EXPECT_TRUE(std::all_of(map.data.begin(), map.data.end(), [](int8_t c) { return c == -1; }));
}

TEST_F(MapMemoryCoreTest, CostmapCellLandsAtItsWorldPosition)
{
  auto costmap = makeCostmap();
  setCost(costmap, 7, 5, 100);

  // Costmap frame at (2, 3), no rotation: (0.25, 0.05) -> world (2.25, 3.05) -> map cell (222, 230)
  memory.integrateCostmap(costmap, {2.0, 3.0, 0.0});
  const auto& map = memory.map();

  EXPECT_EQ(costAt(map, 222, 230), 100);
  EXPECT_EQ(costAt(map, 221, 230), 0);   // neighbour: costmap cell (6, 5), free
  EXPECT_EQ(costAt(map, 215, 230), 0);   // left edge of the 1 x 1 m costmap footprint
  EXPECT_EQ(costAt(map, 214, 230), -1);  // just outside the footprint: never seen
  EXPECT_EQ(costAt(map, 0, 0), -1);
}

TEST_F(MapMemoryCoreTest, CostmapIsRotatedByTheHeading)
{
  auto costmap = makeCostmap();
  setCost(costmap, 7, 5, 100);

  // Heading 90 deg: (0.25, 0.05) rotates to (-0.05, 0.25) -> world (1.95, 3.25) -> map cell (219, 232)
  memory.integrateCostmap(costmap, {2.0, 3.0, M_PI / 2.0});
  const auto& map = memory.map();

  EXPECT_EQ(costAt(map, 219, 232), 100);
  EXPECT_EQ(costAt(map, 222, 230), 0);  // where the obstacle would be without the rotation
}

TEST_F(MapMemoryCoreTest, HigherCostWinsWhenMerging)
{
  auto first = makeCostmap();
  setCost(first, 7, 5, 100);
  memory.integrateCostmap(first, {2.0, 3.0, 0.0});

  auto second = makeCostmap();  // cell (7, 5) now free, cell (6, 5) now 40
  setCost(second, 6, 5, 40);
  memory.integrateCostmap(second, {2.0, 3.0, 0.0});

  auto third = makeCostmap();   // cell (6, 5) now only 20
  setCost(third, 6, 5, 20);
  memory.integrateCostmap(third, {2.0, 3.0, 0.0});

  const auto& map = memory.map();
  EXPECT_EQ(costAt(map, 222, 230), 100);  // a seen obstacle is never erased
  EXPECT_EQ(costAt(map, 221, 230), 40);   // raised 0 -> 40, then not lowered to 20
}

TEST_F(MapMemoryCoreTest, UnknownCostmapCellsLeaveTheMapUnchanged)
{
  auto seen = makeCostmap();
  setCost(seen, 7, 5, 100);
  memory.integrateCostmap(seen, {2.0, 3.0, 0.0});

  memory.integrateCostmap(makeCostmap(-1), {2.0, 3.0, 0.0});   // same area, all unknown
  memory.integrateCostmap(makeCostmap(-1), {-5.0, -5.0, 0.0}); // new area, all unknown

  const auto& map = memory.map();
  EXPECT_EQ(costAt(map, 222, 230), 100);
  EXPECT_EQ(costAt(map, 150, 150), -1);  // (-4.95, -4.95): covered only by unknown cells
}

TEST_F(MapMemoryCoreTest, CostmapHangingOffTheMapEdgeIsClipped)
{
  auto costmap = makeCostmap();
  setCost(costmap, 5, 5, 77);  // centred on (0.05, 0.05)

  // At (19.9, 19.9) most of the costmap is outside the map; (19.95, 19.95) is the last map cell
  memory.integrateCostmap(costmap, {19.9, 19.9, 0.0});

  EXPECT_EQ(costAt(memory.map(), 399, 399), 77);
}

TEST_F(MapMemoryCoreTest, IntegratesFirstCostmapThenOnlyAfterMovingFarEnough)
{
  EXPECT_TRUE(memory.shouldIntegrate({0.0, 0.0, 0.0}, 0.0));

  memory.integrateCostmap(makeCostmap(), {0.0, 0.0, 0.0});

  EXPECT_FALSE(memory.shouldIntegrate({1.0, 1.0, 0.0}, 0.0));  // moved 1.41 m < 1.5 m
  EXPECT_TRUE(memory.shouldIntegrate({1.5, 0.3, 0.0}, 0.0));   // moved 1.53 m
}

TEST_F(MapMemoryCoreTest, NeverIntegratesWhileTurningFast)
{
  EXPECT_FALSE(memory.shouldIntegrate({0.0, 0.0, 0.0}, 0.5));   // even the first costmap
  EXPECT_FALSE(memory.shouldIntegrate({0.0, 0.0, 0.0}, -0.5));  // either direction
  EXPECT_TRUE(memory.shouldIntegrate({0.0, 0.0, 0.0}, 0.2));    // slow turn is fine
}

TEST(YawFromQuaternionTest, RecoversTheHeading)
{
  EXPECT_NEAR(robot::yawFromQuaternion(yawQuaternion(0.0)), 0.0, 1e-9);
  EXPECT_NEAR(robot::yawFromQuaternion(yawQuaternion(M_PI / 2.0)), M_PI / 2.0, 1e-9);
  EXPECT_NEAR(robot::yawFromQuaternion(yawQuaternion(-3.0 * M_PI / 4.0)), -3.0 * M_PI / 4.0, 1e-9);
}
