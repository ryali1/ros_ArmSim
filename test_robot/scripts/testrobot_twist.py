#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import Twist
 
def publish_velocity_commands():
  # Velocity publisher
  vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
  rospy.init_node('testrobot_twist', anonymous=True)
 
  msg = Twist()
  msg.linear.x = .2
  msg.linear.y = .1
  msg.linear.z = 0
  msg.angular.x = 0
  msg.angular.y = 0
  msg.angular.z = .2
 
  rate = rospy.Rate(.5) # 10hz
  while not rospy.is_shutdown():
    vel_pub.publish(msg)
    rate.sleep()
 
if __name__ == '__main__':
  if len(sys.argv) == 1:
    try:
      publish_velocity_commands()      
    except rospy.ROSInterruptException:
      pass
  else:
    print("Usage: rosrun testrobot testrobot_twist.py")
