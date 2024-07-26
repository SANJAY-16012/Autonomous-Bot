#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class HolonomicController:
    def __init__(self):
        rospy.init_node('holonomic_controller')

        # Publisher for each wheel
        self.pub_fl = rospy.Publisher('/front_left_wheel/command', Float64, queue_size=10)
        self.pub_fr = rospy.Publisher('/front_right_wheel/command', Float64, queue_size=10)
        self.pub_rl = rospy.Publisher('/rear_left_wheel/command', Float64, queue_size=10)
        self.pub_rr = rospy.Publisher('/rear_right_wheel/command', Float64, queue_size=10)

        # Subscriber to cmd_vel
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

    def cmd_vel_callback(self, msg):
        # Extract velocity commands
        Vx = msg.linear.x
        Vy = msg.linear.y
        omega = msg.angular.z

        # Calculate wheel velocities
        velocities = self.calculate_wheel_velocities(Vx, Vy, omega)

        # Publish wheel velocities according to specified order
        self.pub_fl.publish(velocities[0])  # Front left
        self.pub_fr.publish(velocities[1])  # Front right
        self.pub_rl.publish(velocities[2])  # Rear left
        self.pub_rr.publish(velocities[3])  # Rear right

    def calculate_wheel_velocities(self, Vx, Vy, omega):
        # Placeholder for actual robot dimensions
        robot_width = 0.5
        robot_length = 0.5

        # Kinematic model for holonomic movement (modifying to match the wheel order)
        wheel_matrix = [
            [1, -1, -(robot_width / 2 + robot_length / 2)],  # Front left
            [-1, -1, -(robot_width / 2 + robot_length / 2)],   # Front right
            [1, 1, -(robot_width / 2 + robot_length / 2)],  # Rear left
            [-1, 1, -(robot_width / 2 + robot_length / 2)]   # Rear right
        ]

        # Compute velocities
        wheel_velocities = [sum(vel * coef for vel, coef in zip([Vx, Vy, omega], row)) for row in wheel_matrix]
        return wheel_velocities

if __name__ == '__main__':
    controller = HolonomicController()
    rospy.spin()
