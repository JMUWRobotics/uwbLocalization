#!/usr/bin/env python
import rospy
import turtle as t
import math
import numpy as np
import sys
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from uwb.msg import AnchorPoint
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

#constants to be set:
numAnchors = 4
refreshRate = 10 #Hz

maxPixelHeight = 800
maxPixelWidth = 1400

minX = -1.1
maxX = 13.1
minY = -5.1
maxY = 5.1

tabLength = 3 # pixels

# truth_pos = [0.5379972271067162, 1.1246135038312388, 1.3059962190302266, 0.41976442986730755, 1.7608554426565388, 2.6605936288136114, 0.6909008300071113, 1.8171807286929316, -1.7764722462193185, -0.9670285524110119, -1.115298895998488] #2024-10-04
# truth_pos = [0.9612933775468444, 5.244348687920962, 0.9795536981918449, -1.231541426834111, -1.5125745508132347, 2.5630841778782996, -1.3580147493403998, 2.9464126755035194, 3.6795168541642895, -0.8523943203633499, 4.1839853667673] #2024-10-09
# truth_pos = [0.5485919070193704, 4.993901372830134, 1.1750767781963551, -1.1405920151470836, -1.3193056298943266, 2.7552602459666953, -0.7883952820540333, 2.3553075035555593, 3.8805322549991774, -1.0939936267180637, 3.506605282922135] #2024-10-10
# truth_pos = [2.7515140957892577, 0.19911237289241318, -1.841073716, 0.524334131958888, 4.304349117652699, 1.826525501, 0.20755819566353723, -1.8171979002872012, 2.1064062733943754, -1.2588366872347918, 2.9713341258014054, 3.3379049925885935, -0.4975648404024999, 2.7376125341719444] #2024-10-11
# truth_pos = [3.973111752299989, -0.9594147173630165, -2.849387649, 1.5480713267048594, -0.8318303239254748, 1.585311376, -0.5917389290585527, -1.6832409232239474, 5.02342321695, -1.9122025109104168, 4.70671266970557, 1.038050879424432, 1.726081552319833, 1.3808325971566497] #2024-10-18
# truth_pos = [1.6083076251209745, -1.5449849059081648, -1.804271695, -0.22007634575127638, -1.8183897373888818, 3.681120840093124, -1.5886253934775008, 3.807612861495501, 3.3822442918908, -0.3798385067377501, 2.4358729085912505] #2024-12-12
# truth_pos = [1.990629770886519, -1.4276541898548727, -2.115592654, 3.02, 0.32, 0.3889631, 1.0094887984269143, 1.5321128719517978, 1.4364287601298051, 0.012774054348750355, 1.9309783196797503, 0.31274290994328013, -2.1625287707225005, 2.4341516409435293, 2.0356466703543674, 3.1363180532654167, -2.4617101085539583] #2024-12-16
# truth_pos = [0.07308885897732406, 0.755268268743298, -2.793196488, 2.4107360657297265, -1.4094878404689524, 0.5625885917565442, -0.5731944689826577, -1.639969119521375, 2.377857367, 2.612723594239688, 1.723072990058047, -0.3134113619833333, -2.6455925340950004, -0.17967479688202104, 1.6819714313935419, 2.21750756963675, -2.631406587184091] #2024-12-17
# truth_pos = [1.1026716613769531, -2.54508087158203, -1.562631, 1.9258915384161737, -1.932897109994898, 1.609126559, 2.796688863170627, 1.9934734833213046, 1.5597461500781835, 3.0697119611410004, -2.5134033965199998, -0.8507831037396838, 0.876577251194853, 3.3394167911369994, 0.21354459881699966, -1.1513145959480002, -1.596852196172925] #2024-12-17-2
# truth_pos = [7.025642611847392, -2.7002245971611956, -0.2571129282330737, 8.358900817506447, -3.327925646499937, 3.090498032, 3.172692210778809, -3.9069828534569218, -1.299596598081667, 1.3097160957994998, -4.322653680633125, 6.640689998591667, -0.5041143839166676, 6.702373272688334, -4.5037317763016675, 1.1435327599863043, 0.8927231077882499] #2024-12-17-3
truth_pos = [7.723912908074214, -0.6909566310010833, -0.27283334192762265, 7.1129580312135365, 0.66368685311541, 2.759643664, 4.156153693527501, -1.4459552730898222, -1.817633689, 2.0833250686940836, -1.8554501980839584, 7.570240385651875, -1.547453711485626, 6.7463577219258335, 2.1461259210349994, 1.2440530280307507, 2.9026115622635] #2024-12-18

#variables:
sf = np.min([maxPixelHeight / (maxY - minY), maxPixelWidth / (maxX - minX)]) # scaling factor pixel/meter

minX *= sf
maxX *= sf
minY *= sf
maxY *= sf

ekf_pos = [0, 0, 0]
ekf_std = [0, 0, 0]

uwb_robot_pos = [0,0,0]
uwb_pos = [0 for i in range(2 * numAnchors)]
uwb_std = [0 for i in range(2 * numAnchors)]

odom_pos = [0, 0, 0]

hector_pos = [0, 0, 0]

def ekfPosCallback(msg: Float64MultiArray):
    global ekf_pos
    ekf_pos = msg.data

def ekfCovCallback(msg: Float64MultiArray):
    global ekf_std
    var = np.array(msg.data)
    var = np.where(var < 0, 0, var)
    ekf_std = np.sqrt(var)

def uwbPosCallback(msg: Float64MultiArray):
    global uwb_robot_pos
    uwb_robot_pos = msg.data
    
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

def drawEllipse(t1, pos, a, b, theta=0.0, steps = 360):
    """
    this function draws an ellipse

    Parameters
    ----------
    t1 : Turtle
        the turtle used to draw the ellipse
    pos : tuple, with two enteries
        the position, where the ellipse is drawn
    a : float
        half of the width of the ellipse
    b : float
        half of the height of the ellipse
    theta : float, optional
        angle by which the ellipse is rotated in radians
    steps : int, optional
        how many individual line segments the ellipse is drawn from
    """
    if a == 0:
        a = sys.float_info.min
    if b == 0:
        b = sys.float_info.min
    stepsize = 2 * math.pi / steps
    cosT = np.cos(theta)
    sinT = np.sin(theta)
    for i in range(steps + 1):
        if i == 0:
            t1.pu()
        else:
            t1.pd()
        x = a * math.cos(i * stepsize)
        y = b * math.sin(i * stepsize)
        x_rot = x * cosT - y * sinT
        y_rot = x * sinT + y * cosT
        t1.setpos((pos[0] + x_rot) * sf, (pos[1] + y_rot) * sf)

def drawCross(t1, pos, size = 2):
    t1.pu()
    t1.setpos(pos[0] * sf - size, pos[1] * sf - size)
    t1.pd()
    t1.setpos(pos[0] * sf + size, pos[1] * sf  + size)
    t1.pu()
    t1.setpos(pos[0] * sf - size, pos[1] * sf + size)
    t1.pd()
    t1.setpos(pos[0] * sf + size, pos[1] * sf - size)
    t1.hideturtle()

def drawNodes(drawingTurtle, pos, std_dev=np.zeros(len(truth_pos)), theta=0.0):
    if len(pos) < 2 * numAnchors:
        return
    posOffset = len(pos) - 2 * numAnchors
    for i in range(posOffset, len(pos), 2):
        drawEllipse(drawingTurtle, (pos[i], pos[i + 1]), std_dev[i], std_dev[i + 1], theta)
        drawCross(drawingTurtle, (pos[i], pos[i + 1]))

def drawRobot(drawingTurtle, runningTurtle, pos, std_dev=[0,0,0]):
    drawEllipse(drawingTurtle, (pos[0], pos[1]), std_dev[0], std_dev[1])
    runningTurtle.setpos(pos[0] * sf, pos[1] * sf)
    runningTurtle.seth(np.rad2deg(pos[2]))
    for i in range(1, int((len(pos) - 2 * numAnchors) / 3)):
        runningTurtle.stamp()
        runningTurtle.setpos(pos[3 * i] * sf, pos[3 * i + 1] * sf)
        runningTurtle.seth(np.rad2deg(pos[3 * i + 2]))

def genTurtle(drawing = True, color="black"):
    tur = t.Turtle()
    if drawing:
        tur.ht()
    tur.color(color)
    return tur

if __name__ == '__main__':
    rospy.init_node('drawScenario')
    rospy.Subscriber("/ekf1_5/pos", Float64MultiArray, ekfPosCallback)
    rospy.Subscriber("/ekf1_5/cov", Float64MultiArray, ekfCovCallback)
    rospy.Subscriber("/uwb1_5/pos", Float64MultiArray, uwbPosCallback)
    rospy.Subscriber("/uwb/anchorPosition", AnchorPoint, anchorPositionCallback)
    rospy.Subscriber("/odom", Odometry, posOdomCallback)
    rospy.Subscriber("/slam_out_pose", PoseStamped, posHectorSlamCallback)
    rate = rospy.Rate(refreshRate)
    t.setup(maxX - minX, maxY - minY)
    t.screensize(maxX - minX, maxY - minY)
    t.setworldcoordinates(minX, minY, maxX, maxY)
    t.ht()
    ekfT = genTurtle(True, "black")
    robotEkfT = genTurtle(False, "black")
    uwbT = genTurtle(True, "red")
    robotUwbT = genTurtle(False, "red")
    odomT = genTurtle(True, "blue")
    robotOdomT = genTurtle(False, "blue")
    hectorT = genTurtle(True, "orange")
    robotHectorT = genTurtle(False, "orange")
    truthT = genTurtle(True, "green")
    robotTruthT = genTurtle(False, "green")
    robotTruthT.pu()
    t.tracer(0)
    t.pu()
    t.setpos(0, 1.1 * minY)
    t.pd()
    t.setpos(0, 1.1 *  maxY)
    t.pu()
    t.setpos(3, 0.95 * maxY)
    t.pd()
    t.write("Y")
    t.pu()
    t.setpos(1.1 * minX, 0)
    t.pd()
    t.setpos(1.1 * maxX, 0)
    t.pu()
    t.setpos(0.95 * maxX, 0)
    t.pd()
    t.write("X")
    t.pu()
    for i in range(int(minX / sf), int(maxX / sf) + 1):
        t.setpos(i * sf, -tabLength)
        t.pd()
        t.setpos(i * sf, tabLength)
        t.pu()
    for i in range(int(minY / sf), int(maxY / sf) + 1):
        t.setpos(-tabLength, i * sf)
        t.pd()
        t.setpos(tabLength, i * sf)
        t.pu()
    drawNodes(truthT, truth_pos)
    drawRobot(truthT, robotTruthT, truth_pos)
    t.tracer(1)
    while True:
        t.tracer(0)
        ekfT.clear()
        drawNodes(ekfT, ekf_pos, ekf_std)
        drawRobot(ekfT, robotEkfT, ekf_pos, ekf_std)
        uwbT.clear()
        # drawNodes(uwbT, uwb_pos, uwb_std, ekf_pos[2])
        drawRobot(uwbT, robotUwbT, uwb_robot_pos)
        odomT.clear()
        drawRobot(odomT, robotOdomT, odom_pos)
        hectorT.clear()
        drawRobot(hectorT, robotHectorT, hector_pos)
        t.tracer(1)
        rate.sleep()