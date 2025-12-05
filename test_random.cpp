#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_demo", ros::init_options::AnonymousName);


    //start ROS spinning thread
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface group("gimbal_arm_controller");

    group.setRandomTarget();

    // Get the random target
    std::vector<double> target = group.getRandomJointValues();

    // Print the random target
    for (double value : target) {
        std::cout << value << std::endl;
    }
    group.move();

    ros::waitForShutdown();

}