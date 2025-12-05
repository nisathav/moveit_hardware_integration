#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

double x_value=0, y_value=0, z_value=0; // Declare global variables
bool new_data_available = false; 

void faceAngleCallback(const geometry_msgs::Point::ConstPtr& msg)
{
  // Store the values in global variables
  x_value = msg->x;
  y_value = msg->y;
  z_value = msg->z;
  new_data_available = true;

  // ROS_INFO("Face Angle Position: x = %f, y = %f, z = %f", msg->x, msg->y, msg->z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    sleep(2.0);

    moveit::planning_interface::MoveGroupInterface group("gimbal_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    //creating publisher
    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    //creating subscriber to /face_angle_position
    ros::Subscriber face_angle_position = nh.subscribe("/face_angle_position", 1, faceAngleCallback);

    ros::Rate loop_rate(125);

    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
    while (ros::ok())
    {
      if (new_data_available){
        // Update the joint group positions using the values from the subscriber
        std::vector<double> joint_group_positions(4);
        joint_group_positions[0] = z_value;  // joint1
        joint_group_positions[1] = x_value;   // joint2
        joint_group_positions[2] = 0.0;   // joint3
        joint_group_positions[3] = y_value;   // joint4
        group.setJointValueTarget(joint_group_positions);
        // Now, you can plan and move to the target
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO("Visualizing plan (joint space goal) %s", success ? "" : "FAILED");
        ROS_INFO("x_value: %f, y_value: %f, z_value: %f", x_value, y_value, z_value);
        // Execute the plan
        success = (group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        new_data_available = false; 
        }
      ros::spinOnce();
      loop_rate.sleep();
    }
      
    // // moving the robot using the last link
    // geometry_msgs::Pose target_pose;
    // // Set position and orientation values for target_pose here...
    // target_pose.orientation.x = 0.157;
    // target_pose.orientation.y = 0.049;
    // target_pose.orientation.z = 0.291;
    // target_pose.orientation.w = 0.942;
    
    // target_pose.position.x = 0.086;
    // target_pose.position.y = 0.110;
    // target_pose.position.z = 1.717;
    // moveit::planning_interface::MoveGroupInterface move_group("gimbal_arm");
    // move_group.setPoseTarget(target_pose, "link_4");
    // // Now, we call the planner to compute the plan and visualize it.
    // // Note that we are just planning, not asking move_group to actually move the robot.
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    // // Visualize the plan in Rviz
    // display_trajectory.trajectory_start = my_plan.start_state_;
    // display_trajectory.trajectory.push_back(my_plan.trajectory_);
    // display_publisher.publish(display_trajectory);
    // /* Sleep to give Rviz time to visualize the plan. */
    // sleep(5.0);
    // // Execute the plan
    // move_group.execute(my_plan);

    //ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
    
    // moving the robot using the end-effector
    // Target position
    // geometry_msgs::Pose target_pose1;
    // target_pose1.orientation.x = 0;
    // target_pose1.orientation.y = 0;
    // target_pose1.orientation.z = 0;
    // target_pose1.orientation.w = 0
    // target_pose1.position.x = 0;
    // target_pose1.position.y = 0;
    // target_pose1.position.z = 1;
    // group.setPoseTarget(target_pose1);

    // visualize the planning
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
    // ROS_INFO("visualizeing plan %s", success.val ? "":"FAILED");

    // move the group arm
    // group.move();

    // moving the robot using individual link
    // Set target for joint values - individual joint control

    ros::shutdown();
    return 0;
}
  