// codes related to movegroup are within the callback function
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

double x_value = 0, y_value = 0, z_value = 0; // Declare global variables
moveit::planning_interface::MoveGroupInterface* move_group_ptr = nullptr;

void faceAnglePositionCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    // Store the values in global variables
    x_value = msg->x;
    y_value = msg->y;
    z_value = msg->z;

    // Update the joint group positions using the values from the subscriber
    std::vector<double> joint_group_positions(4);
    joint_group_positions[0] = 0.0;  // joint1
    joint_group_positions[1] = x_value;   // joint2
    joint_group_positions[2] = z_value;   // joint3
    joint_group_positions[3] = y_value;   // joint4
    move_group_ptr->setJointValueTarget(joint_group_positions);

    // Set the velocity and acceleration scaling to maximum (1.0 = 100%)
    move_group_ptr->setMaxVelocityScalingFactor(1.0);
    move_group_ptr->setMaxAccelerationScalingFactor(1.0);
    
    // Now, you can plan and move to the target
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_ptr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan (joint space goal) %s", success ? "" : "FAILED");
    ROS_INFO("x_value: %f, y_value: %f, z_value: %f", x_value, y_value, z_value);

    // Execute the plan
    success = (move_group_ptr->execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface group("gimbal_arm_controller");
    move_group_ptr = &group; // Store the pointer to the MoveGroup interface


    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    // Creating subscriber to /face_angle_position
    ros::Subscriber face_angle_position = nh.subscribe("/face_angle_position", 10, faceAnglePositionCallback);

    ros::Rate loop_rate(30); // 125 Hz loop rate

    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::shutdown();
    return 0;
}
 