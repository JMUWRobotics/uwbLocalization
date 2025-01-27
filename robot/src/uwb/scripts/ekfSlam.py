#!/usr/bin/env python

import rospy
import numpy as np
import os
from uwb.msg import AnchorPoint
from uwb.msg import Distance
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

clear = lambda: os.system('clear')

#constants to be set:
#Time in between two repetions of the ekf in s
dT = 0.1

#number of anchors deployed:
numAnchors = 4

#number of anchorPoint values will be averaged to get the initial values for the ekf:
numAnchorValuesForInit = 10

#number of repetitions without new data before the program shuts down:
maxRepetitionsWithoutNewData = 100

#sets whether the distance measurements in between the anchors are used as a measurement input for the kalman filter
useInterAnchorDistanceMeas = True

#distance that will be added to every distance measurement, to offset constant error
distanceMeasuremetOffset = -0.5

#the variance of the anchor to anchor distance measurements
distanceVar = 0.005 #0.005 represent a std_dev of ~7cm

#the variance of the anchor position determination measurements
anchorVar = 1

#global constants:
#publisher:
ekfPosPub = rospy.Publisher("/ekf/pos", Float64MultiArray, queue_size=10)
ekfCovPub = rospy.Publisher("/ekf/cov", Float64MultiArray, queue_size=10)

#distance between the two wheels in m
wheelDis = 0.444

#encoder noise const.
K_r = 0
K_l = 0

#global variables:
#distance covered by robot wheels:
u_r = 0
u_l = 0

#ekf slam:
n = 3 + 2 * numAnchors #size of ekf state
m = 2 * numAnchors + int(numAnchors * (numAnchors - 1) / 2) #number of measurements
pos = np.zeros(n) #ekf state: [x_r, y_r, theta, x_t1, y_t1, ... y_tn, y_tn]
cov = np.zeros((n, n)) #ekf covariance

#from uwb measured positions of the anchors, in robot coordinate frame:
anchorPointsPos = [[-1, -1] for i in range(numAnchors)]
anchorPointsVar = [[-1, -1] for i in range(numAnchors)]

#distances from anchor to anchor:
distances = [[-1 for i in range(numAnchors)] for k in range(numAnchors)]
maxDistanceVal = 100

initAnchorPointsPos = [[[], []] for i in range(numAnchors)]

initPhase = True

def distanceCallback(msg: Distance):
    global distances
    i = msg.node1 - 3 # -3, as there are 3 tags
    k = msg.node2 - 3 # -3, as there are 3 tags
    if msg.distance < 0 or msg.distance > maxDistanceVal:
        return
    if i >= 0 and k >= 0:
        distances[i][k] = msg.distance + distanceMeasuremetOffset

def anchorPointCallback(msg: AnchorPoint):
    global anchorPointsPos, anchorPointsVar, initPhase
    if initPhase:
        initAnchorPointsPos[msg.anchorId][0].append(msg.x_pos)
        initAnchorPointsPos[msg.anchorId][1].append(msg.y_pos)
        if checkEnoughValuesExistForAllAnchors():
            initPhase = False
            for i in range(numAnchors):
                anchorPointsPos[i][0] = np.mean(initAnchorPointsPos[i][0])
                anchorPointsPos[i][1] = np.mean(initAnchorPointsPos[i][1])
                anchorPointsVar[i][0] = np.var(initAnchorPointsPos[i][0])
                anchorPointsVar[i][1] = np.var(initAnchorPointsPos[i][1])
    else:
        anchorPointsPos[msg.anchorId][0] = msg.x_pos
        anchorPointsPos[msg.anchorId][1] = msg.y_pos
        anchorPointsVar[msg.anchorId][0] = msg.x_var
        anchorPointsVar[msg.anchorId][1] = msg.y_var

def odomCallback(msg: Odometry):
    global K_r, K_l, u_r, u_l
    K_r = msg.pose.covariance[0]
    K_l = msg.pose.covariance[7]
    u = msg.twist.twist.linear.x
    w = msg.twist.twist.angular.z
    u_r =  (w * wheelDis / 2 + u) * dT
    u_l =  (- w * wheelDis / 2 + u) * dT

def clearMeasurements():
    global anchorPointsPos, anchorPointsVar, distances
    anchorPointsPos = [[-1, -1] for i in range(numAnchors)]
    anchorPointsVar = [[-1, -1] for i in range(numAnchors)]
    distances = [[-1 for i in range(numAnchors)] for k in range(numAnchors)]

def checkMeasurements():
    mat = np.zeros((0, m))
    for i in range(numAnchors):
        if not anchorPointsPos[i][0] == -1:
            tmp = np.zeros(m)
            tmp[2 * i] = 1
            mat = np.vstack((mat, tmp))
            tmp = np.zeros(m)
            tmp[2 * i + 1] = 1
            mat = np.vstack((mat, tmp))
    c = 0
    for i in range(numAnchors):
        for k in range(i + 1, numAnchors):
            if not distances[i][k] == -1 and not distances[k][i] == -1:
                tmp = np.zeros(m)
                tmp[2 * numAnchors + c] = 1
                mat = np.vstack((mat, tmp))
            c += 1
    return mat

def dis(i, k) -> float:
    '''
    returns the distance between two anchors, by averaging the two measurements between them'''
    if i == k:
        return 0
    if i < 0 or i >= numAnchors or k < 0 or k >= numAnchors:
        return -1
    return (distances[i][k] + distances[k][i]) / 2

def checkEnoughValuesExistForAllAnchors() -> bool:
    for i in range(numAnchors):
        if len(initAnchorPointsPos[i][0]) < numAnchorValuesForInit:
            return False
    return True

def ekfStep():
    global pos, cov
    u = 0.5 * (u_l + u_r)
    w = (u_r - u_l) / wheelDis
    sinT = np.sin(pos[2])
    cosT = np.cos(pos[2])
    F = np.identity(n)
    F[2][0] = - u * sinT
    F[2][1] = u * cosT
    L = np.zeros((n, 2))
    L[0][0] = L[0][1] = 0.5 * cosT
    L[1][0] = L[1][1] = 0.5 * sinT
    L[2][0] = L[2][1] = 1 / wheelDis
    Q = np.diag([K_r * np.abs(u_r), K_l * np.abs(u_l)])
    R = np.zeros((m, m))
    R[0 : 2 * numAnchors, 0 : 2 * numAnchors] = anchorVar * np.eye(2 * numAnchors)
    R[2 * numAnchors :, 2 * numAnchors :] = distanceVar * np.eye(m - 2 * numAnchors)
    #predict pos estimation:
#    print("u, w: [", u, ", ", w, "]", sep='')
    pos[0] += u * cosT
    pos[1] += u * sinT
    pos[2] += w
    sinT = np.sin(pos[2])
    cosT = np.cos(pos[2])
    #predict covariance estimation:
    cov = np.dot(np.dot(F, cov), np.transpose(F)) + np.dot(np.dot(L, Q), np.transpose(L))
    #predict measurements:
    z = np.zeros(m) #position of the anchors in robot frame and distance between them
    for i in range(numAnchors): #estimated position of anchors in robot frame:
        z[2 * i : 2 * i + 2] = np.dot(np.array([[cosT, sinT], [-sinT, cosT]]), np.array([pos[2 * i + 3], pos[2 * i + 4]]) - np.array([pos[0], pos[1]]))
    c = 0
    for i in range(numAnchors):
        for k in range(i + 1, numAnchors): #estimated distance between anchors:
            z[2 * numAnchors + c] = np.sqrt((pos[2 * i + 3] - pos[2 * k + 3])**2 + (pos[2 * i + 4] - pos[2 * k + 4])**2)
            c += 1
    #measurement prediction matrix:
    H = np.zeros((m, n))
    tmp_block = np.array([[-cosT, -sinT], [sinT, -cosT]])
    for i in range(numAnchors): # differentiation of position of anchors:
        H[2 * i : 2 * i + 2, 0:2] = tmp_block
        H[2 * i, 2] = -(pos[2 * i + 3] - pos[0]) * sinT + (pos[2 * i + 4] - pos[1]) * cosT
        H[2 * i + 1, 2] = -(pos[2 * i + 3] - pos[0]) * cosT - (pos[2 * i + 4] - pos[1]) * sinT
        H[2 * i : 2 * i + 2, 2 * i + 3 : 2 * i + 5] = -tmp_block
    c = 0
    for i in range(numAnchors):
        for k in range(i + 1, numAnchors): # differentiation of distances between anchors:
            H[2 * numAnchors + c, 2 * i + 3] = (pos[2 * i + 3] - pos[2 * k + 3]) / z[2 * numAnchors + c]
            H[2 * numAnchors + c, 2 * k + 3] = -H[2 * numAnchors + c, 2 * i + 3]
            H[2 * numAnchors + c, 2 * i + 4] = (pos[2 * i + 4] - pos[2 * k + 4]) / z[2 * numAnchors + c]
            H[2 * numAnchors + c, 2 * k + 4] = -H[2 * numAnchors + c, 2 * k + 3]
            c += 1
    #fill measurements in vector:
    y = np.zeros(m)
    y[0 : 2 * numAnchors] = np.reshape(anchorPointsPos, 2 * numAnchors) #measured position of anchors in robot frame
    c = 0
    for i in range(numAnchors):
        for k in range(i + 1, numAnchors): # measured distance between anchors:
            y[2 * numAnchors + c] = dis(i, k)
            c += 1
    #create matrix, that represents, which measurements were successfully retreived in between ekf steps and adapt measurement -prediction, -prediction matrix and -vector:
    G = checkMeasurements()
    z = np.dot(G, z)
    H = np.dot(G, H)
    y = np.dot(G, y)
    #innovation:
    y -= z
#    print("y:", y)
    #innovation covariance:
    S = np.dot(np.dot(H, cov), np.transpose(H)) + np.dot(np.dot(G, R), np.transpose(G))
#    print("S:", S)
    S_inv = np.linalg.inv(S)
#    print("S_inv:", S_inv)
    #kalman gain:
    K = np.dot(np.dot(cov, np.transpose(H)), S_inv)
#    print("K:", K)
    #update pos estimation:
    tmp = np.dot(K, y)
#    print("anpassung:", tmp)
    pos += tmp
    #update covariance prediction:
    cov = np.dot(np.identity(n) - np.dot(K, H), cov)
    clearMeasurements()
    tmp = np.where(G == 1)
    if not np.size(tmp[0]) == 0:
        print("meas:", tmp[1])
    # print("Robot: [", pos[0], ", ", pos[1], ", ", np.rad2deg(pos[2]), "]", sep='')
    # print("Anchors:", pos[3:])

def initEKF():
    global pos, cov
    pos[0] = pos[1] = pos[2] = 0
    cov = np.zeros((3, 3))
    for i in range(numAnchors):
        pos[2 * i + 3] = anchorPointsPos[i][0]
        pos[2 * i + 4] = anchorPointsPos[i][1]
        cov = np.block([[cov, np.zeros((2 * i + 3, 2))], [np.zeros((2, 2* i + 3)), np.diag(anchorPointsVar[i])]])
    print("Initial Values for EKF:")
    print("pos:", pos)
    print("cov:", cov)

if __name__ == '__main__':
    rospy.init_node('ekfSlam')
    rospy.Subscriber("/uwb/anchorPosition", AnchorPoint, anchorPointCallback)
    rospy.Subscriber("/odom", Odometry, odomCallback)
    useInterAnchorDistanceMeas = rospy.get_param("~useInterAnchor", False)
    distanceVar = rospy.get_param("~distanceVar", 0.005)
    anchorVar = rospy.get_param("~anchorVar", 1)
    wheelDis = rospy.get_param("/odometry/wheel_base", 44.4) / 100 # divide by 100 as parameter is in cm not m
    if useInterAnchorDistanceMeas:
        rospy.Subscriber("uwb/distance", Distance, distanceCallback)

    #tmp:
    # anchorPointsPos[0][0] = -0.5917389290585527
    # anchorPointsPos[0][1] = -1.6832409232239474
    # anchorPointsPos[1][0] = 4.70671266970557
    # anchorPointsPos[1][1] = 1.038050879424432
    # anchorPointsPos[2][0] = 5.02342321695
    # anchorPointsPos[2][1] = -1.9122025109104168
    # anchorPointsPos[3][0] = 1.726081552319833
    # anchorPointsPos[3][1] = 1.3808325971566497
    # for i in range(numAnchors):
    #     anchorPointsVar[i][0] = 0.0025
    #     anchorPointsVar[i][1] = 0.0025
    # initPhase = False

    rate = rospy.Rate(1 / dT)
    print("waiting for first positional data from all anchors ...")
    while initPhase:
        rate.sleep()
    initEKF()
    clearMeasurements()
    while not rospy.is_shutdown():
#        clear()
        ekfStep()
        ekfPosMsg = Float64MultiArray()
        ekfPosMsg.data = pos
        ekfCovMsg = Float64MultiArray()
        ekfCovMsg.data = cov.diagonal()
        ekfPosPub.publish(ekfPosMsg)
        ekfCovPub.publish(ekfCovMsg)
        rate.sleep()