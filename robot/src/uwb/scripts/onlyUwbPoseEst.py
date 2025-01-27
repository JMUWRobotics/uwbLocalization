#!/usr/bin/env python

import rospy
import numpy as np
from scipy.optimize import minimize
from uwb.msg import AnchorPoint
from std_msgs.msg import Float64MultiArray

#number of anchors deployed:
numAnchors = 4

anchorPointsPosFromUWB = [[-1, -1] for i in range(numAnchors)]
anchorPointsPosFromEKF = [[-1, -1] for i in range(numAnchors)]

uwbPosPub = rospy.Publisher("/uwb/pos", Float64MultiArray, queue_size=10)

def anchorPointCallback(msg: AnchorPoint):
    global anchorPointsPosFromUWB
    anchorPointsPosFromUWB[msg.anchorId][0] = msg.x_pos
    anchorPointsPosFromUWB[msg.anchorId][1] = msg.y_pos
    if msg.anchorId == numAnchors - 1 and not anchorPointsPosFromEKF[0][0] == -1:
        uwbPosMsg = Float64MultiArray()
        uwbPosMsg.data = estimatePose()
        uwbPosPub.publish(uwbPosMsg)

def ekfPosCallback(msg: Float64MultiArray):
    global anchorPointsPosFromEKF
    for i in range(numAnchors):
        anchorPointsPosFromEKF[i][0] = msg.data[2 * i + 3]
        anchorPointsPosFromEKF[i][1] = msg.data[2 * i + 4]

def squaredErrorAnchorToAnchor(pos):
    retValue = 0
    for i in range(numAnchors):
        retValue += (np.cos(pos[2]) * (anchorPointsPosFromEKF[i][0] - pos[0]) + np.sin(pos[2]) * (anchorPointsPosFromEKF[i][1] - pos[1]) - anchorPointsPosFromUWB[i][0])**2 + (-np.sin(pos[2]) * (anchorPointsPosFromEKF[i][0] - pos[0]) + np.cos(pos[2]) * (anchorPointsPosFromEKF[i][1] - pos[1]) - anchorPointsPosFromUWB[i][1])**2
    return retValue

def estimatePose():
    pos = [0,0,0]
    return minimize(squaredErrorAnchorToAnchor, pos).x

if __name__ == '__main__':
    rospy.init_node('onlyUwbPoseEst')
    rospy.Subscriber("/uwb/anchorPosition", AnchorPoint, anchorPointCallback)
    rospy.Subscriber("/ekf/pos", Float64MultiArray, ekfPosCallback)
    rospy.spin()