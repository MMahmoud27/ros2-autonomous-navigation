#include <algorithm>
#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "planner_core.hpp"

// Test grids use 1 m cells with the corner at (0, 0), so cell (x, y) is centred on (x + 0.5, y + 0.5)
// and expected paths can be read straight off the grid. Test params: cells at cost >= 40 are blocked,
// cost weight 3, blocked goals snap to a free cell within 1 m.

namespace
{

// Pinned here so tuning params.yaml or the defaults never changes what these tests check
robot::PlannerParams testParams()
{
  robot::PlannerParams params;
  params.lethal_cost = 40;
  params.goal_max_cost = 40;  // goals need no more clearance than paths, unless a test says so
  params.cost_weight = 3.0;
  params.goal_snap_radius = 1.0;
  return params;
}

nav_msgs::msg::OccupancyGrid makeGrid(int width, int height, int8_t fill = 0)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "sim_world";
  grid.info.resolution = 1.0f;
  grid.info.width = width;
  grid.info.height = height;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(width * height, fill);
  return grid;
}

void setCost(nav_msgs::msg::OccupancyGrid& grid, int x, int y, int8_t cost)
{
  grid.data.at(y * grid.info.width + x) = cost;
}

int costUnder(const nav_msgs::msg::OccupancyGrid& grid, const robot::Point2D& p)
{
  return grid.data.at(static_cast<int>(p.y) * grid.info.width + static_cast<int>(p.x));
}

bool contains(const std::vector<robot::Point2D>& path, double x, double y)
{
  return std::any_of(path.begin(), path.end(), [&](const robot::Point2D& p) { return p.x == x && p.y == y; });
}

}  // namespace

class PlannerCoreTest : public ::testing::Test
{
protected:
  robot::PlannerCore planner{rclcpp::get_logger("planner_core_test"), testParams()};
};

TEST_F(PlannerCoreTest, StraightLineInAnEmptyGrid)
{
  const auto result = planner.planPath(makeGrid(10, 10), {0.5, 0.5}, {5.5, 0.5});

  ASSERT_EQ(result.status, robot::PlanStatus::kOk);
  ASSERT_EQ(result.path.size(), 6u);  // cells (0,0) .. (5,0)
  EXPECT_DOUBLE_EQ(result.path.front().x, 0.5);
  EXPECT_DOUBLE_EQ(result.path.back().x, 5.5);
  for (const auto& p : result.path) {
    EXPECT_DOUBLE_EQ(p.y, 0.5);
  }
}

TEST_F(PlannerCoreTest, UsesDiagonalMoves)
{
  const auto result = planner.planPath(makeGrid(10, 10), {0.5, 0.5}, {3.5, 3.5});

  ASSERT_EQ(result.status, robot::PlanStatus::kOk);
  ASSERT_EQ(result.path.size(), 4u);  // three diagonal steps: the only shortest route
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_DOUBLE_EQ(result.path[i].x, i + 0.5);
    EXPECT_DOUBLE_EQ(result.path[i].y, i + 0.5);
  }
}

TEST_F(PlannerCoreTest, GoesThroughTheOnlyGapInAWall)
{
  auto grid = makeGrid(10, 10);
  for (int y = 0; y <= 8; ++y) {
    setCost(grid, 5, y, 100);  // wall along x = 5 with a gap at the top, cell (5, 9)
  }

  const auto result = planner.planPath(grid, {2.5, 0.5}, {7.5, 0.5});

  ASSERT_EQ(result.status, robot::PlanStatus::kOk);
  EXPECT_TRUE(contains(result.path, 5.5, 9.5));
  for (const auto& p : result.path) {
    EXPECT_LT(costUnder(grid, p), 40) << "path crosses the wall at (" << p.x << ", " << p.y << ")";
  }
}

TEST_F(PlannerCoreTest, WallWithNoGapGivesNoPath)
{
  auto grid = makeGrid(10, 10);
  for (int y = 0; y < 10; ++y) {
    setCost(grid, 5, y, 100);
  }

  const auto result = planner.planPath(grid, {2.5, 0.5}, {7.5, 0.5});

  EXPECT_EQ(result.status, robot::PlanStatus::kNoPath);
  EXPECT_TRUE(result.path.empty());
}

TEST_F(PlannerCoreTest, DetoursAroundCostlyCellsWhenThatIsCheaper)
{
  // Middle row has cost 30 (passable but expensive) for x = 2..7. Straight through costs
  // 3 + 6 * (1 + 3 * 0.3) = 14.4; dipping into the free row next to it costs 7 + 2 * sqrt(2) = 9.8.
  auto grid = makeGrid(10, 3);
  for (int x = 2; x <= 7; ++x) {
    setCost(grid, x, 1, 30);
  }

  const auto result = planner.planPath(grid, {0.5, 1.5}, {9.5, 1.5});

  ASSERT_EQ(result.status, robot::PlanStatus::kOk);
  for (const auto& p : result.path) {
    EXPECT_EQ(costUnder(grid, p), 0) << "path uses costly cell (" << p.x << ", " << p.y << ")";
  }
}

TEST(PlannerCoreParamsTest, WithoutCostWeightTheShortestPathGoesStraightThrough)
{
  robot::PlannerParams params = testParams();
  params.cost_weight = 0.0;
  robot::PlannerCore planner(rclcpp::get_logger("planner_core_test"), params);
  auto grid = makeGrid(10, 3);
  for (int x = 2; x <= 7; ++x) {
    setCost(grid, x, 1, 30);
  }

  const auto result = planner.planPath(grid, {0.5, 1.5}, {9.5, 1.5});

  ASSERT_EQ(result.status, robot::PlanStatus::kOk);
  EXPECT_EQ(result.path.size(), 10u);  // straight along the middle row
}

TEST_F(PlannerCoreTest, UnknownCellsCountAsFree)
{
  const auto result = planner.planPath(makeGrid(10, 1, -1), {0.5, 0.5}, {9.5, 0.5});

  ASSERT_EQ(result.status, robot::PlanStatus::kOk);
  EXPECT_EQ(result.path.size(), 10u);
}

TEST_F(PlannerCoreTest, StartInsideTheBlockedZoneCanDriveOut)
{
  auto grid = makeGrid(10, 1);
  setCost(grid, 0, 0, 60);  // robot starts close to an obstacle...
  setCost(grid, 1, 0, 50);  // ...and the next cell is still blocked, but less so

  const auto result = planner.planPath(grid, {0.5, 0.5}, {9.5, 0.5});

  ASSERT_EQ(result.status, robot::PlanStatus::kOk);
  EXPECT_EQ(result.path.size(), 10u);
}

TEST_F(PlannerCoreTest, StartCannotMoveDeeperIntoTheBlockedZone)
{
  auto grid = makeGrid(10, 1);
  setCost(grid, 0, 0, 60);
  setCost(grid, 1, 0, 70);  // the only way forward is closer to the obstacle

  const auto result = planner.planPath(grid, {0.5, 0.5}, {9.5, 0.5});

  EXPECT_EQ(result.status, robot::PlanStatus::kNoPath);
}

TEST_F(PlannerCoreTest, BlockedGoalMovesToTheNearestFreeCell)
{
  auto grid = makeGrid(10, 1);
  setCost(grid, 9, 0, 100);  // goal clicked on an obstacle; cell 8, 1 m away, is free

  const auto result = planner.planPath(grid, {0.5, 0.5}, {9.5, 0.5});

  ASSERT_EQ(result.status, robot::PlanStatus::kOk);
  EXPECT_DOUBLE_EQ(result.path.back().x, 8.5);
}

TEST(PlannerCoreParamsTest, GoalsNeedMoreClearanceThanPaths)
{
  // Cell 9 costs 30: fine to drive through (below 40), but with goal_max_cost 20 too close to park
  // on, so the goal moves to cell 8 (cost 10), 1 m away
  auto grid = makeGrid(10, 1);
  setCost(grid, 9, 0, 30);
  setCost(grid, 8, 0, 10);
  robot::PlannerParams params = testParams();

  params.goal_max_cost = 40;
  robot::PlannerCore lenient(rclcpp::get_logger("planner_core_test"), params);
  const auto kept = lenient.planPath(grid, {0.5, 0.5}, {9.5, 0.5});

  params.goal_max_cost = 20;
  robot::PlannerCore strict(rclcpp::get_logger("planner_core_test"), params);
  const auto moved = strict.planPath(grid, {0.5, 0.5}, {9.5, 0.5});

  ASSERT_EQ(kept.status, robot::PlanStatus::kOk);
  EXPECT_DOUBLE_EQ(kept.path.back().x, 9.5);
  ASSERT_EQ(moved.status, robot::PlanStatus::kOk);
  EXPECT_DOUBLE_EQ(moved.path.back().x, 8.5);
}

TEST_F(PlannerCoreTest, GoalDeepInsideAnObstacleIsRejected)
{
  auto grid = makeGrid(10, 1);
  for (int x = 7; x <= 9; ++x) {
    setCost(grid, x, 0, 100);  // nearest free cell (6) is 3 m from the goal, beyond the 1 m snap radius
  }

  const auto result = planner.planPath(grid, {0.5, 0.5}, {9.5, 0.5});

  EXPECT_EQ(result.status, robot::PlanStatus::kGoalBlocked);
  EXPECT_TRUE(result.path.empty());
}

TEST_F(PlannerCoreTest, PointsOutsideTheMapAreRejected)
{
  const auto grid = makeGrid(10, 10);

  EXPECT_EQ(planner.planPath(grid, {-1.0, 0.5}, {5.5, 0.5}).status, robot::PlanStatus::kStartOutsideMap);
  EXPECT_EQ(planner.planPath(grid, {0.5, 0.5}, {20.0, 0.5}).status, robot::PlanStatus::kGoalOutsideMap);
}

TEST_F(PlannerCoreTest, DoesNotCutDiagonallyBetweenTwoBlockedCells)
{
  auto grid = makeGrid(3, 3);
  setCost(grid, 1, 0, 100);
  setCost(grid, 0, 1, 100);  // the start's only way out is squeezing diagonally between these

  const auto result = planner.planPath(grid, {0.5, 0.5}, {2.5, 2.5});

  EXPECT_EQ(result.status, robot::PlanStatus::kNoPath);
}
