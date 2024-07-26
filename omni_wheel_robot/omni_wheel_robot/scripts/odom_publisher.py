

#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, Pose, Twist, Vector3
from std_msgs.msg import Int32
import tf
import numpy as np

class HolonomicOdometry:
    def __init__(self):
        rospy.init_node('holonomic_odometry')

        # Publishers
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()

        # Subscribers
        rospy.Subscriber('/frontleft', Int32, self.frontleft_callback)
        rospy.Subscriber('/frontright', Int32, self.frontright_callback)
        rospy.Subscriber('/rearleft', Int32, self.rearleft_callback)
        rospy.Subscriber('/rearright', Int32, self.rearright_callback)

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        # Odometry position
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # Encoder ticks
        self.frontleft_ticks = 0
        self.frontright_ticks = 0
        self.rearleft_ticks = 0
        self.rearright_ticks = 0

        # Previous encoder ticks
        self.last_frontleft_ticks = 0
        self.last_frontright_ticks = 0
        self.last_rearleft_ticks = 0
        self.last_rearright_ticks = 0

        # Wheel parameters (example values, replace with your own)
        self.ticks_per_revolution = 135  # Number of encoder ticks per wheel revolution
        self.wheel_radius = 0.0635  # Wheel radius in meters
        self.L = 0.54  # Distance between left and right wheels
        self.W = 0.54 # Distance between front and back wheels

        self.rate = rospy.Rate(20)  # 20 Hz to match encoder topic rate

    def frontleft_callback(self, msg):
        self.frontleft_ticks = msg.data

    def frontright_callback(self, msg):
        self.frontright_ticks = msg.data

    def rearleft_callback(self, msg):
        self.rearleft_ticks = msg.data

    def rearright_callback(self, msg):
        self.rearright_ticks = msg.data

    def calculate_odometry(self):
        self.current_time = rospy.Time.now()

        # Calculate change in encoder ticks
        delta_frontleft_ticks = self.frontleft_ticks - self.last_frontleft_ticks
        delta_frontright_ticks = self.frontright_ticks - self.last_frontright_ticks
        delta_rearleft_ticks = self.rearleft_ticks - self.last_rearleft_ticks
        delta_rearright_ticks = self.rearright_ticks - self.last_rearright_ticks

        # Update previous tick counts
        self.last_frontleft_ticks = self.frontleft_ticks
        self.last_frontright_ticks = self.frontright_ticks
        self.last_rearleft_ticks = self.rearleft_ticks
        self.last_rearright_ticks = self.rearright_ticks

        # Calculate wheel velocities in radians/sec
        wheel_velocities = (2 * np.pi * np.array([delta_frontleft_ticks, delta_frontright_ticks, delta_rearleft_ticks, delta_rearright_ticks])) / (self.ticks_per_revolution * self.rate.sleep_dur.to_sec())
        
        # Convert wheel velocities to linear velocities
        linear_velocities = wheel_velocities * self.wheel_radius
        
        # Calculate the robot's velocity
        vx = (linear_velocities[0] - linear_velocities[1] + linear_velocities[2] - linear_velocities[3]) / 4
        vy = (-linear_velocities[0] - linear_velocities[1] + linear_velocities[2] + linear_velocities[3]) / 4
        vth = (-linear_velocities[0] - linear_velocities[1] - linear_velocities[2] - linear_velocities[3]) / (4 * (self.L + self.W))
        
        # Calculate change in position
        dt = (self.current_time - self.last_time).to_sec()
        delta_x = (vx * np.cos(self.th) - vy * np.sin(self.th)) * dt
        delta_y = (vx * np.sin(self.th) + vy * np.cos(self.th)) * dt
        delta_th = vth * dt

        # Update the robot's position
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # Create quaternion from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

        # Publish the transform over tf
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.),
            odom_quat,
            self.current_time,
            "base_link",
            "odom"
        )

        # Publish the odometry message
        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"

        # Set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        # Set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        # Publish the message
        self.odom_pub.publish(odom)

        self.last_time = self.current_time

    def run(self):
        while not rospy.is_shutdown():
            self.calculate_odometry()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        odom_node = HolonomicOdometry()
        odom_node.run()
    except rospy.ROSInterruptException:
        pass