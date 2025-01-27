#!/usr/bin/env python

import time
import rospy
import numpy as np
from uwb.msg import Distance
from std_msgs.msg import Bool
from std_msgs.msg import UInt8

#constants to be set:
numNodes = 3

printToFile = False
NUM = 500
continuousRunning = False
outFilename = "..."

#global variables:
distances = [[[] for i in range(numNodes)] for j in range(numNodes)]

resetPub = rospy.Publisher("/uwb/reset", Bool, queue_size=10)
resetMsg = Bool(True)
activatePub = rospy.Publisher("/uwb/activate", UInt8, queue_size=10)
activateMsg = UInt8(numNodes)

def initNodes():
    time.sleep(1)
    activatePub.publish(activateMsg)
    time.sleep(1)
    print("initlized all nodes")

def finish():
    print("num:", [[np.size(distances[i][j]) for i in range(numNodes)] for j in range(numNodes)])
    print("mean:", [[np.mean(distances[i][j]) for i in range(numNodes)] for j in range(numNodes)])
    print("std:", [[np.std(distances[i][j]) for i in range(numNodes)] for j in range(numNodes)])
    if printToFile:
        outFile = open(outFilename, "w")
        outLine = ""
        for d in distances[0][1]:
            outLine += str(d)
            outLine += ", "
        outLine = outLine[:-2]
        outFile.write(outLine)
        outFile.close()
    resetPub.publish(resetMsg)
    rospy.signal_shutdown("got all values")

def distanceCallback(msg: Distance):
    if msg.distance < 0 or msg.distance > 50:
        return
    global distances
    i = msg.node1
    k = msg.node2
    distances[i][k].append(msg.distance)
    l = np.size(distances[i][k])
    if l == NUM + 1:
        distances[i][k].pop(0)
    if i == numNodes - 1 and k == numNodes - 2:
        if continuousRunning:
            print("mean:", [[np.mean(distances[i][j]) for i in range(numNodes)] for j in range(numNodes)])
        else:
            print(l, "/", NUM)
    if not continuousRunning and i == numNodes - 1 and k == numNodes - 2 and l == NUM:
        finish()

if __name__ == '__main__':
    rospy.init_node('getAvgDistance')
    rospy.Subscriber("/uwb/distance", Distance, distanceCallback)
    initNodes()
    rospy.spin()
