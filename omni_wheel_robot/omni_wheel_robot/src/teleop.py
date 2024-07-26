# #!/usr/bin/env python

# import rospy
# from geometry_msgs.msg import Twist

# class CmdVelRepublisher:
#     def __init__(self):
#         rospy.init_node('cmd_vel_republisher', anonymous=True)
#         self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
#         self.cmd_vel_pub = rospy.Publisher('/cmd_vel_republished', Twist, queue_size=10)

#     def cmd_vel_callback(self, msg):
#         # Simply republish the received Twist message
#         self.cmd_vel_pub.publish(msg)

#     def spin(self):
#         rospy.spin()

# if __name__ == '__main__':
#     try:
#         republisher = CmdVelRepublisher()
#         republisher.spin()
#     except rospy.ROSInterruptException:
#         pass





