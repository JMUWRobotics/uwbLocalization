#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from uwb.msg import AnchorPoint
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import csv

#constants to be set:
numAnchors = 4

filename = "/home/martin/UWBMasterThesis/uwb/src/uwb/data/trajectories/2024-12-18/"

#global variables:
ekf_pos = [0 for i in range(2 * numAnchors + 3)]
ekf_std = [0 for i in range(2 * numAnchors + 3)]

uwb_robot_pos = [0,0,0]
uwb_pos = [0 for i in range(2 * numAnchors)]
uwb_std = [0 for i in range(2 * numAnchors)]

odom_pos = [0, 0, 0]

hector_pos = [0, 0, 0]

ekf_pos_list = []
ekf_std_list = []
uwb_robot_pos_list = []
uwb_pos_list = []
odom_pos_list = []
hector_pos_list = []

def ekfPosCallback(msg: Float64MultiArray):
    global ekf_pos
    # ekf_pos = msg.data
    for i in range(2 * numAnchors + 3):
        ekf_pos[i] = msg.data[i]

def ekfCovCallback(msg: Float64MultiArray):
    global ekf_std
    var = np.array(msg.data)
    var = np.where(var < 0, 0, var)
    std = np.sqrt(var)
    # ekf_std = std
    for i in range(2 * numAnchors + 3):
        ekf_std[i] = std[i]

def uwbPosCallback(msg: Float64MultiArray):
    global uwb_robot_pos
    # uwb_robot_pos = msg.data
    uwb_robot_pos[0] = msg.data[0]
    uwb_robot_pos[1] = msg.data[1]
    uwb_robot_pos[2] = msg.data[2]
    
def posOdomCallback(msg: Odometry):
    global odom_pos
    q = msg.pose.pose.orientation
    odom_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, euler_from_quaternion([q.x, q.y, q.z, q.w])[2]]

def anchorPositionCallback(msg: AnchorPoint):
    global uwb_pos, uwb_std
    sinT = np.sin(ekf_pos[2])
    cosT = np.cos(ekf_pos[2])
    uwb_pos[2 * msg.anchorId] = cosT * msg.x_pos - sinT * msg.y_pos + ekf_pos[0]
    uwb_pos[2 * msg.anchorId + 1] = sinT * msg.x_pos + cosT * msg.y_pos + ekf_pos[1]
    uwb_std[2 * msg.anchorId] = np.sqrt(msg.x_var)
    uwb_std[2 * msg.anchorId + 1] = np.sqrt(msg.y_var)

def posHectorSlamCallback(msg: PoseStamped):
    global hector_pos
    q = msg.pose.orientation
    hector_pos = [msg.pose.position.x, msg.pose.position.y, euler_from_quaternion([q.x, q.y, q.z, q.w])[2]]    

def saveData(event):
    global ekf_pos_list, ekf_std_list, uwb_robot_pos_list, uwb_pos_list, odom_pos_list, hector_pos_list
    ekf_pos_list.append(ekf_pos.copy())
    ekf_std_list.append(ekf_std.copy())
    uwb_robot_pos_list.append(uwb_robot_pos.copy())
    uwb_pos_list.append(uwb_pos.copy())
    odom_pos_list.append(odom_pos)
    hector_pos_list.append(hector_pos)

if __name__ == '__main__':
    rospy.init_node('evalEKF')
    rospy.Subscriber("/ekf/pos", Float64MultiArray, ekfPosCallback)
    rospy.Subscriber("/ekf/cov", Float64MultiArray, ekfCovCallback)
    rospy.Subscriber("/uwb/pos", Float64MultiArray, uwbPosCallback)
    rospy.Subscriber("/uwb/anchorPosition", AnchorPoint, anchorPositionCallback)
    rospy.Subscriber("/odom", Odometry, posOdomCallback)
    rospy.Subscriber("/slam_out_pose", PoseStamped, posHectorSlamCallback)
    filename += rospy.get_param("~filenameExt", "")
    filename += ".csv"
    rospy.Timer(rospy.Duration(0.1), saveData)
    rospy.spin()
    # print(ekf_pos_list)
    # print(ekf_std_list)
    # print(uwb_robot_pos_list)
    # print(uwb_pos_list)
    # print(odom_pos_list)
    # print(hector_pos_list)
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file, delimiter=',')
        for i in range(len(ekf_pos_list)):
            writer.writerow(ekf_pos_list[i] + ekf_std_list[i] + uwb_robot_pos_list[i] + uwb_pos_list[i] + odom_pos_list[i] + hector_pos_list[i])
    print("finished writing to file")