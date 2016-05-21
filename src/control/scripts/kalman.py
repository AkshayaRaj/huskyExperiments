#!/usr/bin/env python

# Author : Akshay Raj Dayal
# Date :19/04/2106
# Extended Kalman Filter Implementation for Viso2 + IMU
# TODO : fix covariance for Viso measurement , use standard XSENS/PixHawk Covariance 

import math
import rospy
import time
import numpy as np
import threading
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


last_viso_seq = 0
viso_seq =0;
z = np.array([[0.0],[0.0]])
dt = 0.01
cmd = np.array ((0.0,0.0))

def ekf_viso(state,P,Qk,Rk):
    global dt
    state_n,P_n = system_update(state,P,Qk,dt)
    state_n,P_n = measurement_update(state,P,Rk,dt)
    #rospy.spin()
    #rospy.loginfo("Executed..")
    return (state_n,P_n)





def getViso(msg):
    global last_viso_seq
    global z
    #dt = getTime(msg) - last_viso_time

    viso_odom = msg;
    lock = threading.Lock()
    lock.acquire()
    viso_seq = msg.header.seq
    z[0][0] = msg.pose.pose.position.z
    z[1][0] = msg.pose.pose.position.x
    lock.release()
    last_viso_seq  = msg.header.seq



def getTime(msg):
    time = msg.header.stamp.secs + msg.header.stamp.nsecs*10^-9
    return time

def getCmd(msg):
    global cmd
    lock = threading.Lock()
    lock.acquire()
    cmd[0] = msg.twist.linear.x
    cmd[1] = msg.twist.angular.z
    print cmd

    lock.release



def system_update(state,P,Qk,dt):
    state[3] = cmd[0]
    state[4] = cmd[1]
    if state[3] == 0:
        state[3] =0.0
    if state[4] ==0:
        state[4]=0.0

    state_pre = np.array(( (state[0]-state[3]*dt*np.sin(state[2])),\
    state[1]+state[3]*dt*np.cos(state[2]),state[2]+state[4]*dt ,state[3],state[4] ));

    Ak = np.array([  [1.0,0.0,-state[3]*dt*np.cos(state[2]),-dt*np.sin(state[2]),0.0],\
                     [0.0,1.0,-state[3]*dt*np.sin(state[2]),dt*np.cos(state[2]),0.0],\
                     [0.0,0.0,1.0,0.0,dt],\
                     [0.0,0.0,0.0,1.0,0.0],\
                     [0.0,0.0,0.0,0.0,1.0] ])
    rospy.loginfo(Ak)
    P_pre = np.dot(np.dot(Ak,P), Ak.T) + Qk
    return (state_pre,P_pre)

def measurement_update(state,P,Rk,dt):
    global viso_seq
    global last_viso_seq
    USE_VISO = 1
    H = np.array([ [1.0,0.0,0.0,0.0,0.0],[0.0,1.0,0.0,0.0,0.0] ])
    h_z = np.array([ [state[0]-state[3]*dt*math.sin(state[2])],\
                     [state[1]+state[3]*dt*math.cos(state[2])] ])
     # Find Kalman Gain
     #K = P*H'*(H*P*H'+Rk)^-1
    innovation = np.dot(H,np.dot(P,H.T)) + Rk
    K = np.dot(P,np.dot(H.T,np.linalg.inv(innovation)))
    state_post = state + np.dot(K,(z - h_z))
    P_post = P - np.dot(K,np.dot(H,P))

    if USE_VISO == 0 or viso_seq == last_viso_seq:
        state_post = state
        P_post = P

    last_viso_seq = viso_seq
    return (state_post,P_post)




if __name__ == "__main__":

    Q = np.diag((0.01 ,0.01, 0.01, 0.03, 0.03 ))
    state = np.array((0.0, 0.0, 0.0,0.0,0.0))
    P = np.diag((0.01,0.01,0.01,0.01,0.01))
    Qk = Q*dt
    R_viso = np.diag((0.1,0.1))
    Rk = R_viso*dt

    rospy.init_node('ekf_viso')
    rospy.Subscriber("/stereo_odometer/odometry", Odometry, getViso)
    rospy.Subscriber("/cmd_vel_stamped", TwistStamped, getCmd)
    pub=rospy.Publisher('/stereo_odometer/odometry/ekf',Odometry,queue_size=10)


    while 1:
        t = time.time()
        state,P = ekf_viso(state,P,Qk,Rk)
        #rospy.loginfo(state)
        diff = time.time() - t
        #rospy.spin()
        if diff<dt:
            time.sleep(dt-diff)
