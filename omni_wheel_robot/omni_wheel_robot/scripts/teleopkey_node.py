# #!/usr/bin/env python

# import rospy
# from geometry_msgs.msg import Twist
# from geometry_msgs.msg import PoseStamped
# from tf.transformations import euler_from_quaternion, quaternion_from_euler
# import math

# class RobotController:
#     def __init__(self):
#         rospy.init_node('robot_controller')
#         rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
#         self.pub_pose = rospy.Publisher('/robot_pose', PoseStamped, queue_size=10)
        
#         self.robot_pose = PoseStamped()
#         self.robot_pose.header.frame_id = 'map'  # Set the frame ID to 'map' or 'odom' based on your setup
#         self.robot_pose.pose.position.x = 0.0
#         self.robot_pose.pose.position.y = 0.0
#         self.robot_pose.pose.orientation.w = 1.0
        
#         self.rate = rospy.Rate(10)  # Update rate (10 Hz)

#     def cmd_vel_callback(self, msg):
#         linear_speed = msg.linear.x  # Linear speed (m/s)
#         angular_speed = msg.angular.z  # Angular speed (rad/s)

#         # Update robot's pose based on cmd_vel
#         dt = 1.0 / self.rate.sleep_dur.to_sec()
#         theta = euler_from_quaternion([
#             self.robot_pose.pose.orientation.x,
#             self.robot_pose.pose.orientation.y,
#             self.robot_pose.pose.orientation.z,
#             self.robot_pose.pose.orientation.w])[2]

#         self.robot_pose.pose.position.x += linear_speed * math.cos(theta) * dt
#         self.robot_pose.pose.position.y += linear_speed * math.sin(theta) * dt
#         theta += angular_speed * dt

#         q = quaternion_from_euler(0, 0, theta)
#         self.robot_pose.pose.orientation.x = q[0]
#         self.robot_pose.pose.orientation.y = q[1]
#         self.robot_pose.pose.orientation.z = q[2]
#         self.robot_pose.pose.orientation.w = q[3]

#         # Publish updated pose
#         self.pub_pose.publish(self.robot_pose)

#         def cmd_vel_callback(self, msg):
#             rospy.loginfo("Received cmd_vel: linear=%f, angular=%f", msg.linear.x, msg.angular.z)
#     # Add your pose update logic here


#     def run(self):
#         rospy.spin()

# if __name__ == '__main__':
#     controller = RobotController()
#     controller.run()



#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from math import cos, sin

class HolonomicController:
    def __init__(self):
        rospy.init_node('holonomic_controller')

        # Joint names for omni-directional robot (adjust based on your URDF)
        self.joint_names = ['front_left_wheel_joint', 'front_right_wheel_joint', 'rear_left_wheel_joint', 'rear_right_wheel_joint']

        # Create a publisher to send joint states to RViz
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=30)

        # Subscribe to /cmd_vel topic to receive Twist messages
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        # Initialize joint state message
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = self.joint_names
        self.joint_state_msg.position = [0.0] * len(self.joint_names)  # Initialize joint positions

    def cmd_vel_callback(self, msg):
        # Calculate wheel velocities based on twist command
        linear_x = msg.linear.x  # Forward/backward velocity
        linear_y = msg.linear.y  # Left/right velocity (strafing)
        angular_z = msg.angular.z  # Angular velocity (yaw)

        # Assuming holonomic omni-directional robot
        wheel_velocities = [
            linear_x - linear_y - angular_z,
            linear_x + linear_y - angular_z,
            linear_x + linear_y + angular_z,
            linear_x - linear_y + angular_z
        ]

        # Update joint positions based on wheel velocities
        dt = 0.4  # Update rate (adjust as needed)
        for i in range(len(self.joint_names)):
            self.joint_state_msg.position[i] += wheel_velocities[i] * dt

        # Publish joint state to RViz
        self.joint_state_msg.header.stamp = rospy.Time.now()
        self.joint_state_pub.publish(self.joint_state_msg)

def main():
    controller = HolonomicController()
    rospy.spin()

if __name__ == '__main__':
    main()
