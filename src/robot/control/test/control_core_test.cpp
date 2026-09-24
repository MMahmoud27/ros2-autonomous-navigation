#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "control_core.hpp"

// Unless a test says otherwise the robot's axle is at the origin facing +x, with the default params:
// lookahead 1.5 m, speed 0.8 m/s, max turn 1.0 rad/s, goal tolerance 0.3 m, turn on the spot beyond
// 1.0 rad, slow down within 1.5 m of the goal but not below 25% speed.
// Pure pursuit: for a target at (x, y) in the robot frame at distance L, curvature = 2y / L^2 and
// angular = linear * curvature.

namespace
{

// n + 1 evenly spaced points from a to b
std::vector<robot::Point2D> line(robot::Point2D a, robot::Point2D b, int n)
{
  std::vector<robot::Point2D> points;
  for (int k = 0; k <= n; ++k) {
    points.push_back({a.x + (b.x - a.x) * k / n, a.y + (b.y - a.y) * k / n});
  }
  return points;
}

// A path along the direction (0.8, 0.6), with points 1 m apart up to 5 m away. The first point at
// least 1.5 m away is (1.6, 1.2), exactly 2 m away.
std::vector<robot::Point2D> pathUpAndLeft()
{
  return line({0.0, 0.0}, {4.0, 3.0}, 5);
}

}  // namespace

class ControlCoreTest : public ::testing::Test
{
protected:
  robot::ControlCore control{rclcpp::get_logger("control_core_test")};
};

TEST_F(ControlCoreTest, EmptyPathMeansStandStill)
{
  const auto cmd = control.computeCommand({}, {});

  EXPECT_EQ(cmd.linear, 0.0);
  EXPECT_EQ(cmd.angular, 0.0);
  EXPECT_FALSE(control.findLookaheadPoint({}, {0.0, 0.0}).has_value());
}

TEST_F(ControlCoreTest, PathStraightAheadDrivesStraightAtFullSpeed)
{
  const auto cmd = control.computeCommand(line({0.0, 0.0}, {5.0, 0.0}, 50), {});

  EXPECT_DOUBLE_EQ(cmd.linear, 0.8);
  EXPECT_DOUBLE_EQ(cmd.angular, 0.0);
}

TEST_F(ControlCoreTest, TargetToTheLeftCurvesLeft)
{
  // Target (1.6, 1.2), L = 2: curvature = 2 * 1.2 / 4 = 0.6, angular = 0.8 * 0.6 = 0.48
  const auto cmd = control.computeCommand(pathUpAndLeft(), {});

  EXPECT_DOUBLE_EQ(cmd.linear, 0.8);
  EXPECT_NEAR(cmd.angular, 0.48, 1e-9);
}

TEST_F(ControlCoreTest, TargetToTheRightCurvesRight)
{
  auto path = pathUpAndLeft();
  for (auto& p : path) {
    p.y = -p.y;
  }

  const auto cmd = control.computeCommand(path, {});

  EXPECT_NEAR(cmd.angular, -0.48, 1e-9);
}

TEST_F(ControlCoreTest, UsesTheRobotHeading)
{
  // Facing along (0.8, 0.6), the same target is dead ahead
  const auto cmd = control.computeCommand(pathUpAndLeft(), {0.0, 0.0, std::atan2(0.6, 0.8)});

  EXPECT_DOUBLE_EQ(cmd.linear, 0.8);
  EXPECT_NEAR(cmd.angular, 0.0, 1e-9);
}

TEST_F(ControlCoreTest, TargetBehindTurnsOnTheSpot)
{
  // Path heads back and to the left: target (-1.6, 0.8) is 153 deg off heading
  const auto cmd = control.computeCommand(line({0.0, 0.0}, {-3.2, 1.6}, 4), {});

  EXPECT_EQ(cmd.linear, 0.0);
  EXPECT_DOUBLE_EQ(cmd.angular, 1.0);
}

TEST(ControlCoreParamsTest, TurnRateLimitSlowsDownInsteadOfWideningTheCurve)
{
  robot::ControlParams params;
  params.linear_speed = 2.0;  // 2.0 * curvature 0.6 = 1.2 rad/s, over the 1.0 limit
  robot::ControlCore control(rclcpp::get_logger("control_core_test"), params);

  const auto cmd = control.computeCommand(pathUpAndLeft(), {});

  EXPECT_DOUBLE_EQ(cmd.angular, 1.0);
  EXPECT_NEAR(cmd.linear, 1.0 / 0.6, 1e-9);  // same curvature (0.6), just slower
}

TEST_F(ControlCoreTest, SlowsDownApproachingTheGoal)
{
  // Goal 1.0 m away: 0.8 * 1.0 / 1.5
  EXPECT_NEAR(control.computeCommand(line({0.0, 0.0}, {1.0, 0.0}, 10), {}).linear, 0.8 / 1.5, 1e-9);
  // Goal 0.35 m away: 0.8 * 0.35 / 1.5 would be 0.19, but the floor is 25% of 0.8
  EXPECT_NEAR(control.computeCommand({{0.0, 0.0}, {0.35, 0.0}}, {}).linear, 0.2, 1e-9);
}

TEST_F(ControlCoreTest, StopsWithinGoalTolerance)
{
  const auto cmd = control.computeCommand({{0.0, 0.0}, {0.2, 0.0}}, {});

  EXPECT_EQ(cmd.linear, 0.0);
  EXPECT_EQ(cmd.angular, 0.0);
}

TEST_F(ControlCoreTest, LookaheadNeverPicksPointsAlreadyPassed)
{
  // Robot halfway along a 5 m path: the start (2 m behind) is far enough away but already passed.
  // 3.5 is 1.45 m ahead (too close), 3.6 is 1.55 m ahead.
  const auto target = control.findLookaheadPoint(line({0.0, 0.0}, {5.0, 0.0}, 50), {2.05, 0.0});

  ASSERT_TRUE(target.has_value());
  EXPECT_NEAR(target->x, 3.6, 1e-9);
}

TEST_F(ControlCoreTest, NearTheEndTheLookaheadIsTheLastPoint)
{
  const auto target = control.findLookaheadPoint(line({0.0, 0.0}, {1.0, 0.0}, 10), {0.0, 0.0});

  ASSERT_TRUE(target.has_value());
  EXPECT_NEAR(target->x, 1.0, 1e-9);
}
