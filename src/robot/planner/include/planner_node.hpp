#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    // Idle until a goal arrives, then navigating until it is reached or given up on
    enum class State { kWaitingForGoal, kNavigating };

    // Declares the A* parameters (defaults overridden by params.yaml) and returns their values
    robot::PlannerParams loadParams();

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr goal);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);

    // Timer: while navigating, ends the trip on arrival or timeout, otherwise replans on the latest map
    void timerCallback();

    // Plans from the robot to the goal and publishes the result (an empty path if planning fails)
    void planAndPublish();

    // Publishes an empty path, which stops the controller, and goes back to waiting for a goal
    void stopNavigating(const std::string& reason);

    void publishPath(const std::vector<robot::Point2D>& points);

    // The robot turns about its wheel axle, lidar_to_axle_ metres behind the odometry (lidar) frame
    std::optional<robot::Point2D> axlePosition() const;

    robot::PlannerCore planner_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    State state_ = State::kWaitingForGoal;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    nav_msgs::msg::Odometry::SharedPtr odom_;
    robot::Point2D goal_;    // where the user clicked
    robot::Point2D target_;  // where the latest path ends: the goal, or where it was moved to
    rclcpp::Time goal_received_time_;
    std::optional<robot::PlanStatus> last_status_;  // so outcomes are logged only when they change

    double goal_tolerance_;
    double timeout_s_;
    double lidar_to_axle_;
};

#endif
