// This code generates the possibles values for geometry_msgs::Pose target_pose;
#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "target_pose_finder");
  ros::NodeHandle node_handle;

  // Remove the AsyncSpinner and spinner.start() lines

  // Load the robot model
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();

  // Create a robot state instance
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));

  // Set the current state of the robot
  robot_state->setToDefaultValues();

  // Get the name of the last link
  const std::vector<std::string>& link_names = robot_state->getRobotModel()->getLinkModelNames();
  const std::string& last_link_name = link_names.back();

  // Get the current transform of the last link
  const Eigen::Isometry3d& last_link_transform = robot_state->getGlobalLinkTransform(last_link_name);

  // Convert the transform to a geometry_msgs::Pose
  geometry_msgs::Pose target_pose;
  target_pose.position.x = last_link_transform.translation().x();
  target_pose.position.y = last_link_transform.translation().y();
  target_pose.position.z = last_link_transform.translation().z();

  Eigen::Quaterniond quaternion(last_link_transform.linear());
  target_pose.orientation.w = quaternion.w();
  target_pose.orientation.x = quaternion.x();
  target_pose.orientation.y = quaternion.y();
  target_pose.orientation.z = quaternion.z();

  ROS_INFO_STREAM("Target pose:\n"
                  "Position: (" << target_pose.position.x << ", " << target_pose.position.y << ", " << target_pose.position.z << ")\n"
                  "Orientation: (" << target_pose.orientation.w << ", " << target_pose.orientation.x << ", " << target_pose.orientation.y << ", " << target_pose.orientation.z << ")");

  ros::spin(); // Use ros::spin() instead of AsyncSpinner

  return 0;
}