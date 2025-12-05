// This code generates the position of the last link of the robot

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "output_last_link_state");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

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

  // Publish the robot state to RViz
  ros::NodeHandle rviz_node_handle;
  ros::Publisher robot_state_publisher = rviz_node_handle.advertise<moveit_msgs::DisplayRobotState>("robot_state", 1);

  moveit_msgs::DisplayRobotState robot_state_msg;
  robot_state_msg.state.is_diff = false;
  moveit::core::robotStateToRobotStateMsg(*robot_state, robot_state_msg.state);

  ROS_INFO_STREAM("Last link name: " << last_link_name);
  ROS_INFO_STREAM("Last link transform:\n" << last_link_transform.matrix());

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    robot_state_publisher.publish(robot_state_msg);
    loop_rate.sleep();
  }

  spinner.stop();
  return 0;
}