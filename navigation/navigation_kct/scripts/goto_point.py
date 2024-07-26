#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from tf.transformations import quaternion_from_euler

# Define positions and orientations for each named zone
zones = [
    {'name': 'start_zone', 'position': (2.2770, -2.1918), 'orientation': (0.0, 0.0, 0.68, 0.72)},
    {'name': 'area2', 'position': (2.4771, 3.5153), 'orientation': (0.0, 0.0, 0.99, 0.04)},
    {'name': 'area2_ramp', 'position': (-1.0937, 3.7695), 'orientation': (0.0, 0.0, 0.76, 0.64)},
    {'name': 'area3', 'position': (1.0778, 7.0399), 'orientation': (0.0, 0.0, 0.06, 0.99)}
]

def send_goal(position, orientation):
    # Create action client for move_base
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Create a goal object
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = position[0]
    goal.target_pose.pose.position.y = position[1]
    goal.target_pose.pose.position.z = 0.0  # Assuming 2D navigation
    goal.target_pose.pose.orientation.x = orientation[0]
    goal.target_pose.pose.orientation.y = orientation[1]
    goal.target_pose.pose.orientation.z = orientation[2]
    goal.target_pose.pose.orientation.w = orientation[3]

    # Send the goal
    client.send_goal(goal)
    client.wait_for_result()

    # Check if goal was successful
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal reached successfully")
        return True
    else:
        rospy.loginfo("Failed to reach goal")
        return False

def navigate_to_zones():
    # Initialize node
    rospy.init_node('sequential_waypoint_navigation')

    current_zone_index = 0
    num_zones = len(zones)

    while not rospy.is_shutdown() and current_zone_index < num_zones:
        zone = zones[current_zone_index]
        rospy.loginfo(f"Sending goal to {zone['name']}")

        # Send goal to current zone
        goal_reached = send_goal(zone['position'], zone['orientation'])

        if goal_reached:
            # Check if the current zone is 'area3'
            if zone['name'] == 'area3':
                rospy.loginfo("Reached the final zone (area3). Stopping navigation.")
                break  # Exit the loop if 'area3' is reached

            # Move to the next zone
            current_zone_index += 1
        else:
            rospy.logwarn("Failed to reach goal. Retrying...")

if __name__ == '__main__':
    try:
        navigate_to_zones()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")
