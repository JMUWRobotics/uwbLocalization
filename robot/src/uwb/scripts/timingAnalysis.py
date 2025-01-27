#!/usr/bin/env python

import rospy
from uwb.msg import TimingResp

# constants to be set:
numResp = 6
numWaitTimes = 5

# global variables:
firstRun = True
sucResp = [[0 for k in range(numResp)] for i in range(numWaitTimes)]

def timingRespCallback(msg: TimingResp):
    global sucResp, firstRun
    index = int(msg.curWaitTime / 1000000) - 1
    if index == 3:
        firstRun = False
    if not firstRun and index == 4:
        end()
    sucResp[index][msg.numSucResp - 1] += 1

def end():
    print("One full cycle is done.")
    print(sucResp)
    rospy.signal_shutdown("Full cacle done.")
    exit()

if __name__ == '__main__':
    rospy.init_node('timingAnalysis')
    rospy.Subscriber("/timingResp", TimingResp, timingRespCallback)
    rospy.spin()