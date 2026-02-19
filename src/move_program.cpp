#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>


int main(int argc, char * argv[])
{
 // Initialize ROS and create the Node
 rclcpp::init(argc, argv);
 auto const node = std::make_shared<rclcpp::Node>(
   "move_program",
   rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
 );


 // Create a ROS logger
 auto const logger = rclcpp::get_logger("move_program");


 // create a MoveGroupInterface object
 moveit::planning_interface::MoveGroupInterface MoveGroupInterface(node, "tanerb");

 // Explicitly set the end-effector link (tip of the kinematic chain)
 MoveGroupInterface.setEndEffectorLink("end_Link");

 // Relax goal tolerances so IK solutions near the target are accepted
 MoveGroupInterface.setGoalPositionTolerance(0.01);      // 1 cm
 MoveGroupInterface.setGoalOrientationTolerance(0.1);    // ~6 deg - wider needed due to joint6 tight limits (-28° to +15°)

 MoveGroupInterface.allowReplanning(true);
 MoveGroupInterface.setNumPlanningAttempts(5);

 // Move to the "home" named state defined in the SRDF (guaranteed reachable).
 // To move to a custom pose instead, use setPositionTarget(x, y, z) for
 // position-only (orientation free), which is more reliable than setPoseTarget
 // because joint6 has very tight limits (-28° to +15°) that make many
 // orientation+position combinations impossible for KDL to solve.
 //
 // Known reachable workspace from TF: e.g. setPositionTarget(0.1, 0.09, 0.35)
 MoveGroupInterface.setNamedTarget("home");
//  MoveGroupInterface.setPositionTarget(0.1, 0.09, 0.35);

 // Plan the motion
 MoveGroupInterface.setPlanningTime(10.0);
 moveit::planning_interface::MoveGroupInterface::Plan plan1;
 auto const success = static_cast<bool>(MoveGroupInterface.plan(plan1));
 if (success)
 {
   MoveGroupInterface.execute(plan1);
   RCLCPP_INFO(logger, "Planning succeeded");
 }
 else
 {
   RCLCPP_ERROR(logger, "Planning failed");
   return 1;
 }


 // Shutdown ROS
 rclcpp::shutdown();
 return 0;
}
