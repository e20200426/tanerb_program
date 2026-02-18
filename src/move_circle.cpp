#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>


//for converting euler angles to quaternions
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


 // Create a ROS logger
 auto const logger = rclcpp::get_logger("move_circle");


 // create a MoveGroupInterface object
 moveit::planning_interface::MoveGroupInterface MoveGroupInterface(node, "tanerb");


 // define the target pose
 tf2::Quaternion tf2_quat;
 //in radians
 tf2_quat.setRPY(0, 0, -3.14); // Roll, Pitch, Yaw
 // Convert to quaternion
 geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);


 // Set the target pose
 geometry_msgs::msg::Pose target_pose;
 target_pose.orientation = msg_quat;
 target_pose.position.x = 0.1;
 target_pose.position.y = 0.1;
 target_pose.position.z = 0.4;
  MoveGroupInterface.setPoseTarget(target_pose);


 // Plan the motion
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
