#!/usr/bin/env python

# Author : Akshay Raj Dayal
# Date :19/04/2106
# This code adds a timestanp value to cmd_vel messages using simulated clock time . Its useful for Huksy cmd_vel
# which is processed without a timestamp by default 

import rospy
import tf.transformations
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
i=1

pub=rospy.Publisher('cmd_vel_stamped',TwistStamped,queue_size=10)

def callback(msg):
    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
    twist=TwistStamped()
    twist.twist.linear=msg.linear
    twist.twist.angular=msg.angular
    twist.header.stamp=rospy.get_rostime()
    twist.header.seq=i+1
    pub.publish(twist)
    # Do velocity processing here:
    # Use the kinematics of your robot to map linear and angular velocities into motor commands


    # Then set your wheel speeds (using wheel_left and wheel_right as examples)

def listener():
    rospy.init_node('cmd_vel_listener')
    rospy.Subscriber("/cmd_vel", Twist, callback)
    pub=rospy.Publisher('cmd_vel_stamped',TwistStamped,queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    listener()
