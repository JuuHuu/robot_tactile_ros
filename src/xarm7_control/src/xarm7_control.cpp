#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = rclcpp::Node::make_shared
  (
    "xarm7_moveit_planner",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // create MoveGroupInterface for planning group "xarm7"
  moveit::planning_interface::MoveGroupInterface move_group(node, "xarm7");


  // define a target pose
  tf2::Quaternion q;
  geometry_msgs::msg::Pose target_pose;
  q.setRPY(3.14159,0,0);
 
  target_pose.orientation= tf2::toMsg(q);
  target_pose.position.x = 0.3; // in meters
  target_pose.position.y = 0.0; // in meters
  target_pose.position.z = 0.3; // in meters


  move_group.setPoseTarget(target_pose);

  // plan to the target
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  auto success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success) {
    RCLCPP_INFO(node->get_logger(), "Plan found, executing...");
    // move_group.execute(my_plan);
  } else {
    RCLCPP_WARN(node->get_logger(), "Planning failed");
  }

  rclcpp::shutdown();
  return 0;
}
