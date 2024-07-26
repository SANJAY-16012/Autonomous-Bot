#!/usr/bin/env python

import rospy
from motor_control.srv import OneInt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math

class MechanismController:
    def __init__(self):
        rospy.init_node('mechanism_controller')

        print("Entering")
        
        # Subscribe to the robot's odometry topic to get its current position
        rospy.Subscriber("/odom", Odometry, self.current_pose_callback)
        
        # Subscribe to the goal topic to get the goal position set in RViz
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_pose_callback)
        
        # Service client for the OneInt service
        rospy.wait_for_service('send_one_int')
        self.send_one_int = rospy.ServiceProxy('send_one_int', OneInt)

        self.rate = rospy.Rate(10)
        self.goal_position = None
        self.current_position = None
        self.mechanism_started = False

    def current_pose_callback(self, msg):
        self.current_position = msg.pose.pose

    def goal_pose_callback(self, msg):
        self.goal_position = msg.pose
        self.mechanism_started = False  # Reset mechanism state when a new goal is received

    def calculate_distance(self, pos1, pos2):
        return math.sqrt(
            (pos1.position.x - pos2.position.x) ** 2 +
            (pos1.position.y - pos2.position.y) ** 2
        )

    def run(self):
        try:
            print("into run function)")
            while not rospy.is_shutdown():
                if self.goal_position and self.current_position and not self.mechanism_started:
                    distance_to_goal = self.calculate_distance(self.current_position, self.goal_position)
                    if distance_to_goal <= 2.0:
                        rospy.loginfo("Calling OneInt service with argument 10")
                        # Call the OneInt service with argument 10
                        try:
                            response = self.send_one_int(10)
                            if response.success:
                                rospy.loginfo("OneInt service called successfully")
                            else:
                                rospy.logwarn("OneInt service call failed")
                        except rospy.ServiceException as e:
                            rospy.logerr(f"Service call failed: {e}")

                        self.mechanism_started = True  # Avoid repeated service calls

                self.rate.sleep()

        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    controller = MechanismController()
    controller.run()
