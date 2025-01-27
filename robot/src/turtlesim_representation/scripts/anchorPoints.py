#!/usr/bin/env python
import rospy
import turtle as t
import math
import numpy as np
import sys
from uwb.msg import AnchorPoint
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

#constants to be set:
numAnchors = 4

maxPixelHeight = 800
maxPixelWidth = 1400

minX = -2
maxX = 4
minY = -3
maxY = 3

tabLength = 3 # pixels

#variables:
sf = np.min([maxPixelHeight / (maxY - minY), maxPixelWidth / (maxX - minX)]) # scaling factor pixel/meter

minX *= sf
maxX *= sf
minY *= sf
maxY *= sf

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

def genTurtle(drawing = True, color="black"):
    tur = t.Turtle()
    if drawing:
        tur.ht()
    tur.color(color)
    return tur

if __name__ == '__main__':
    rospy.init_node('testNode')
    rospy.Subscriber("/uwb/anchorPosition", AnchorPoint, anchorPositionCallback)
    rospy.Subscriber("/slam_out_pose", PoseStamped, posHectorSlamCallback)
    t.setup(maxX - minX, maxY - minY)
    t.screensize(maxX - minX, maxY - minY)
    t.setworldcoordinates(minX, minY, maxX, maxY)
    t.ht()
    tur = (genTurtle(color="red"), genTurtle(color="green"), genTurtle(color="blue"), genTurtle(color="black"))
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
    t.tracer(1)
    rate = rospy.Rate(10)
    while True:
        t.tracer(0)
        for i in range(len(tur)):
            if not len(x[i]) == 0:
                tur[i].clear()
                for x_i, y_i in zip(x[i], y[i]):
                    drawCross(tur[i], (x_i, y_i))
                x_m = np.mean(x[i])
                y_m = np.mean(y[i])
                x_std = np.std(x[i])
                y_std = np.std(y[i])
                drawCross(tur[i], (x_m, y_m), 4)
                drawEllipse(tur[i], (x_m, y_m), x_std, y_std)
                print("Anchor ", i, ": meanX: ", x_m, "\tmeanY: ", y_m, "\tstdX: ", x_std, "\tstdY: ", y_std, sep='')
        t.tracer(1)
        rate.sleep()
