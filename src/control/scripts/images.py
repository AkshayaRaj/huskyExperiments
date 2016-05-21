#!/usr/bin/env python

# Author : Akshay Raj Dayal
# Date :21/06/2106
# This code uses ROS message filter's Approximate Sync Policy to extract stereo image messages
# from a bagfile as image frames

# TODO : Add Inter message lower bound . Ref http://wiki.ros.org/message_filters/ApproximateTime

import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import signal
import message_filters
import os

def rosimg2cv(ros_image):
        try:
            #frame=CvBridge.imgmsg_to_cv2(ros_image,ros_image.encoding)
            frame=CvBridge().imgmsg_to_cv2(ros_image,ros_image.encoding)

        except CvBridgeError as e:
            rospy.logerr(e)
            rospy.loginfo("CvBridge error")

        return frame


def callback(image_left, image_right):
    if image_left.header.stamp != image_right.header.stamp:
        rospy.loginfo("OMG.. we have a sync issue !  contact ard.astro@gmail.com :P ")
        rospy.loginfo("Left: "+str(image_left.header.stamp)+" Right: "+str(image_right.header.stamp))
    else:
        rospy.loginfo(image_left.header.seq)

    cv2.imwrite("./left/left_"+str(image_left.header.seq)+".jpg",rosimg2cv(image_left))
    cv2.imwrite("./right/right_"+str(image_left.header.seq)+".jpg",rosimg2cv(image_right))

    rospy.loginfo("Wrote image: "+"left_"+str(image_left.header.seq)+"| right_"+str(image_right.header.seq))


def listener():

    rospy.init_node('images_out')

    left_image_sub = message_filters.Subscriber('/camera/left/image_raw', Image)
    right_image_sub = message_filters.Subscriber('/camera/right/image_raw', Image)

    left_info_sub = message_filters.Subscriber('/camera/left/camera_info', CameraInfo)
    right_info_sub = message_filters.Subscriber('/camera/right/camera_info', CameraInfo)

#    pub=rospy.Publisher('cmd_vel_stamped',TwistStamped,queue_size=10)

    ts = message_filters.ApproximateTimeSynchronizer([left_image_sub, right_image_sub], 10,0.1)
    ts.registerCallback(callback)

    if not os.path.exists('left'):
        os.makedirs('left')

    if not os.path.exists('right'):
        os.makedirs('right')


    rospy.spin()

if __name__ == '__main__':
    listener()
