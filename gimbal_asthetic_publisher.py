#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Int32
import random
import time
class GimbalCmdPublisher:
    def __init__(self):
        rospy.init_node('gimbal_cmd_publisher', anonymous=True)
        self.face_angle_pub = rospy.Publisher('/face_angle_position', Point, queue_size=10)
        self.focus_position_pub = rospy.Publisher('/set_focus_position', Int32, queue_size=1)
        self.rate = rospy.Rate(125)  # 125 Hz
        self.last_change_time = time.time()
        self.current_yaw = 0.0

    def send_focus_position(self, position):
        msg = Int32()
        msg.data = position
        self.focus_position_pub.publish(msg)
        rospy.loginfo(f"Published focus position: {position}")

    def update_yaw(self):
        current_time = time.time()
        if current_time - self.last_change_time >= 10.0:  # Check if 2 seconds have passed
            self.current_yaw = random.uniform(0.00, 3.14)  # Generate new random yaw
            self.last_change_time = current_time  # Update the last change time

    def run(self):
        while not rospy.is_shutdown():
            self.update_yaw()  # Check and update yaw if necessary
            result_radians = Point()
            result_radians.x = self.current_yaw  # Use the current yaw value
            result_radians.y = 0.0  # pitch
            result_radians.z = 0.0  # linear actuator
            #self.send_focus_position(2000)  # 0-4095 zoom position
            print(f"angles: {result_radians}")
            self.face_angle_pub.publish(result_radians)
            self.rate.sleep()

if __name__ == '__main__':
    gimbal_cmd_publisher = GimbalCmdPublisher()
    gimbal_cmd_publisher.run()