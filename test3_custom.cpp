#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    sleep(2.0);

    moveit::planning_interface::MoveGroupInterface group("gimbal_arm_controller");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    //creating publisher
    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

    std::vector<double> joint_group_positions(4);
    joint_group_positions[0] = -0.5;  // joint1
    joint_group_positions[1] = 3.0;   // joint2
    joint_group_positions[2] = 0.0;   // joint3
    joint_group_positions[3] = 0.0;   // joint4
    group.setJointValueTarget(joint_group_positions);
    // Now, you can plan and move to the target
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan (joint space goal) %s", success ? "" : "FAILED");
    // Execute the plan
    success = (group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ros::shutdown();
    return 0;
}
