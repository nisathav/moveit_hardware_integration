#!/usr/bin/env python3

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class GimbalCommandClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/gimbal_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.client.wait_for_server()

    def send_gimbal_command(self, positions):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["joint5", "joint6", "joint7", "joint8"]
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(3.0)
        
        goal.trajectory.points.append(point)
        
        self.client.send_goal(goal)
        self.client.wait_for_result()

# Usage
if __name__ == '__main__':
    rospy.init_node('gimbal_command_client')
    client = GimbalCommandClient()
    
    # Example: Move all joints to 0.5 radians
    rospy.loginfo("Sending gimbal command...")
    client.send_gimbal_command([0.0, 3.0, 0.0, 0.0])

    rospy.spin()