#include <memory>
#include <cmath>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

// for converting euler angles to quaternions
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>


int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "move_circle",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Spin node in background so MoveIt callbacks are processed
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("move_circle");

  // Create a MoveGroupInterface object
  moveit::planning_interface::MoveGroupInterface move_group(node, "tanerb");
  std::string end_effector_link = move_group.getEndEffectorLink();

  move_group.allowReplanning(true);
  move_group.setGoalPositionTolerance(0.001);
  move_group.setGoalOrientationTolerance(0.01);
  move_group.setMaxAccelerationScalingFactor(0.5);
  move_group.setMaxVelocityScalingFactor(0.5);

  // ── Step 0: move to "home" named pose ────────────────────────────────────
  move_group.setNamedTarget("home");
  moveit::planning_interface::MoveGroupInterface::Plan plan_home;
  bool success = static_cast<bool>(move_group.plan(plan_home));
  if (success)
  {
    move_group.execute(plan_home);
    RCLCPP_INFO(logger, "Reached home pose");
  }
  else
  {
    RCLCPP_ERROR(logger, "Failed to plan to home pose");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  // ── Step 1: move to the circle start pose ────────────────────────────────
  tf2::Quaternion tf2_quat;
  tf2_quat.setRPY(0, 0, -M_PI); // Roll, Pitch, Yaw
  geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);

  geometry_msgs::msg::Pose start_pose;
  start_pose.orientation = msg_quat;
  start_pose.position.x = 0.1;
  start_pose.position.y = 0.1;
  start_pose.position.z = 0.4;

  move_group.setPoseTarget(start_pose, end_effector_link);

  moveit::planning_interface::MoveGroupInterface::Plan plan_to_start;
  success = static_cast<bool>(move_group.plan(plan_to_start));
  if (success)
  {
    move_group.execute(plan_to_start);
    RCLCPP_INFO(logger, "Reached circle start pose");
  }
  else
  {
    RCLCPP_ERROR(logger, "Failed to plan to start pose");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  // ── Step 2: build circular waypoints (Y-Z plane) ─────────────────────────
  const double radius    = 0.05;          // metres
  const double center_y  = start_pose.position.y;
  const double center_z  = start_pose.position.z;
  const double step_rad  = 0.05;         // angular resolution (rad)

  std::vector<geometry_msgs::msg::Pose> waypoints;

  // The starting point is already at angle 0 (cos(0)=1 → y=center_y+r)
  // so we generate points from 0 to 2π inclusive to close the circle.
  for (double th = 0.0; th <= 2.0 * M_PI + step_rad / 2.0; th += step_rad)
  {
    geometry_msgs::msg::Pose wp = start_pose;
    wp.position.y = center_y + radius * std::cos(th);
    wp.position.z = center_z + radius * std::sin(th);
    waypoints.push_back(wp);
  }

  // ── Step 3: compute and execute Cartesian path ────────────────────────────
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double eef_step      = 0.01;  // interpolation resolution (m)
  const double jump_threshold = 0.0;  // disable jump detection
  double fraction = 0.0;

  const int max_tries = 10;
  for (int attempts = 0; attempts < max_tries && fraction < 1.0; ++attempts)
  {
    fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    RCLCPP_INFO(logger, "Cartesian path attempt %d: %.1f%% planned", attempts + 1, fraction * 100.0);
  }

  if (fraction >= 1.0)
  {
    RCLCPP_INFO(logger, "Full circular path computed. Executing...");
    moveit::planning_interface::MoveGroupInterface::Plan circle_plan;
    circle_plan.trajectory_ = trajectory;
    move_group.execute(circle_plan);
    RCLCPP_INFO(logger, "Circle motion complete");
  }
  else
  {
    RCLCPP_ERROR(logger, "Cartesian path planning failed (%.1f%% after %d tries)", fraction * 100.0, max_tries);
  }

  // Shutdown ROS
  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
