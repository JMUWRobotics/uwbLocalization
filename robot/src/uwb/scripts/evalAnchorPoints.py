#!/usr/bin/env python
import rospy
import math
import numpy as np
import sys
from uwb.msg import AnchorPoint
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

#constants to be set:
numAnchors = 4

#global variables:
x = [[] for i in range(numAnchors)]
y = [[] for i in range(numAnchors)]

hector_pos = [0, 0, 0]

def anchorPositionCallback(msg: AnchorPoint):
    global x,y
    sinT = np.sin(hector_pos[2])
    cosT = np.cos(hector_pos[2])
    x_m = msg.x_pos
    y_m = msg.y_pos
    x[msg.anchorId].append(cosT * x_m - sinT * y_m + hector_pos[0])
    y[msg.anchorId].append(sinT * x_m + cosT * y_m + hector_pos[1])

def posHectorSlamCallback(msg: PoseStamped):
    global hector_pos
    q = msg.pose.orientation
    hector_pos = [msg.pose.position.x, msg.pose.position.y, euler_from_quaternion([q.x, q.y, q.z, q.w])[2]]    

if __name__ == '__main__':
    rospy.init_node('evalAnchorPoints')
    rospy.Subscriber("/uwb/anchorPosition", AnchorPoint, anchorPositionCallback)
    rospy.Subscriber("/slam_out_pose", PoseStamped, posHectorSlamCallback)
    rospy.spin()
    i = 1
    for a,b in zip(x,y):
        print("x", i, " = [", sep = '', end='')
        for c in a:
            if not c == a[-1]:
                print(c, ",", sep='', end='')
            else:
                print(c, "];", sep='')
        print("y", i, " = [", sep = '', end='')
        for c in b:
            if not c == b[-1]:
                print(c, ",", sep='', end='')
            else:
                print(c, "];", sep='')
        i += 1
