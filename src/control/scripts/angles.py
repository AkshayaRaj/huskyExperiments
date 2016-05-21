#!/usr/bin/env python


# Author : Akshay Raj Dayal
# Date :15/04/2106
# Print Euler angles from ros quaternion orientations


import rospy
import math
import tf.transformations
from sensor_msgs.msg import Imu

i=1

pub=rospy.Publisher('/mavros/imu/data/angles',Imu,queue_size=10)

def callback(msg):
    #rospy.loginfo("Received an Imu message message!")
    #rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    #rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
    imu=Imu()
    imu=msg
    quaternion = (
    msg.orientation.x,
    msg.orientation.y,
    msg.orientation.z,
    msg.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    imu.orientation.x = roll*180/math.pi
    imu.orientation.y = pitch*180/math.pi
    imu.orientation.z = yaw*180/math.pi
    rospy.loginfo(yaw*180/math.pi)
    pub.publish(imu)
    # Do velocity processing here:
    # Use the kinematics of your robot to map linear and angular velocities into motor commands


    # Then set your wheel speeds (using wheel_left and wheel_right as examples)

def listener():
    rospy.init_node('cmd_vel_listener')
    rospy.Subscriber("/mavros/imu/data", Imu, callback)
    #pub=rospy.Publisher('/mavros/imu/data/angles',Imu,queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    listener()
