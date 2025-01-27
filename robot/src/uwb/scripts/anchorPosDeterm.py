#!/usr/bin/env python

import rospy
import numpy as np
from scipy.optimize import minimize
import time
import os
import copy
from uwb.msg import Distance
from std_msgs.msg import UInt8
from uwb.msg import AnchorPoint

clear = lambda: os.system('clear')

# constants to be set:
# id of anchor and tags must be the actual hardware ids:
anchors = [2555942, 4063280, 3473456, 3145773] #2555942, 4063280, 3473456, 3145773
tags = [2162733, 3407920, 4259888] # back, front-right, front-left; must be 3 tags

distanceMeasuremetOffset = -0.5

distanceFrontWheelAxisToFrontUWBAxis = 0.016

d = 0.6 #side length of the uwb-tag triangle on the robot in m

posAnchorVar = 1    #variance of the determined Anchor position, if there is only one sample -> no sample based variance available

maxDistanceVal = 100  #maximum distance, that is accepted as reasonable

# global constants:
nodes = copy.copy(tags)
nodes.extend(anchors)

# global variables:
distances = [[-1 for i in range(len(nodes))] for j in range(len(nodes))]

tagX = [-(d * np.sqrt(3) / 2 - distanceFrontWheelAxisToFrontUWBAxis), distanceFrontWheelAxisToFrontUWBAxis, distanceFrontWheelAxisToFrontUWBAxis]   #-0.504, 0.016, 0.016
tagY = [0, -d/2, d/2]                                                                                                                               # 0.000, 0.300,-0.300

posAnchors = [[-1, -1] for i in range(len(anchors))]

lastNode1Idx = -1

# publisher to control the uwb nodes:
activatePub = rospy.Publisher("uwb/activate", UInt8, queue_size=10)
activateMsg = UInt8(len(nodes))

# publisher to publish the processed uwb data:
anchorPointPub = rospy.Publisher("uwb/anchorPosition", AnchorPoint, queue_size=10)

def initNodes():
    time.sleep(1)
    activatePub.publish(activateMsg)
    time.sleep(1)
    print("initlized all nodes")

def distanceCallback(msg: Distance):
    global distances, lastNode1Idx
    i = msg.node1
    k = msg.node2
    if i < lastNode1Idx:
        cycle()
    lastNode1Idx = i
    distances[i][k] = msg.distance + distanceMeasuremetOffset

def checkDistanceValuesAreIn(anId) -> bool:
    '''
    checks whether in the last cycle all necessary distance measurements, to estimate the position of the anchor given by anId, have successfully been retreived and are not obviously wrong.
    '''
    for i in range(len(tags)):
        if distances[i][anId] < 0 or distances[anId][i] < 0 or distances[i][anId] > maxDistanceVal or distances[anId][i] > maxDistanceVal:
            return False
    return True

def clearDistance():
    global distances
    distances = [[-1 for i in range(len(nodes))] for j in range(len(nodes))]

def cycle():
    clear()
    # print("\n\n\n")
    for i in range(len(tags), len(nodes)):
        if not checkDistanceValuesAreIn(i):
            print("Anchor ", nodes[i], " (", i, "): no good values.", sep='')
            continue
        anchorIdx = i - len(tags)
        # print status:
        print("Anchor ", nodes[i], " (", i, "):", sep='')
        print("d0", i, ": ", dis(0, i), sep='')
        print("d1", i, ": ", dis(1, i), sep='')
        print("d2", i, ": ", dis(2, i), sep='')
        posAnchors[anchorIdx] = estimateAnchorPosition(anchorIdx)
        print("x_pos: ", posAnchors[anchorIdx][0], sep='')
        print("y_pos: ", posAnchors[anchorIdx][1], sep='')
        print("bearing: ", np.rad2deg(np.arctan2(posAnchors[anchorIdx][1], posAnchors[anchorIdx][0])), sep ='')
        print("distance: ", np.sqrt(posAnchors[anchorIdx][0]**2 + posAnchors[anchorIdx][1]**2), sep ='')
        anchorPointMsg = AnchorPoint()
        anchorPointMsg.anchorId = anchorIdx
        anchorPointMsg.isCorrected = False
        anchorPointMsg.x_pos = posAnchors[anchorIdx][0]
        anchorPointMsg.y_pos = posAnchors[anchorIdx][1]
        anchorPointMsg.x_var = posAnchorVar
        anchorPointMsg.y_var = posAnchorVar
        anchorPointPub.publish(anchorPointMsg)
    clearDistance()

def dis(i, k):
    if i == k:
        return 0
    return (distances[i][k] + distances[k][i]) / 2

def squaredErrorTagsToAnchor(pos, anchorIdx):
    retValue = 0
    for k in range(len(tags)):
        retValue += ((pos[0] - tagX[k])**2 + (pos[1] - tagY[k])**2 - dis(k, anchorIdx + len(tags))**2)**2
        # retValue += (dis(k, anchorIdx + len(tags)) - np.sqrt((pos[0] - tagX[k])**2 + (pos[1] - tagY[k])**2))**2
    return retValue
    # return np.sqrt(retValue)

def estimateAnchorPosition(anchorIdx):
    pos = [0, 0]
    return minimize(squaredErrorTagsToAnchor, pos, args=(anchorIdx,)).x

if __name__ == '__main__':
    rospy.init_node('anchorPosDeterm')
    rospy.Subscriber("/uwb/distance", Distance, distanceCallback)
    if (rospy.get_param("/initNodes", 'True')):
        initNodes()
    rospy.spin()