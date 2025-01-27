import os
import numpy as np

scanFolder = "2024_12_18_17_58_43"
numMidPoses = 2
numTripods = 4

def EulerToMatrix4(pos, ang, angInDeg=False):
    '''converts euler angles into a 4x4 matrix'''
    if angInDeg:
        ang = np.deg2rad(ang)
    mat = np.zeros((4,4))
    sx = np.sin(ang[0])
    cx = np.cos(ang[0])
    sy = np.sin(ang[1])
    cy = np.cos(ang[1])
    sz = np.sin(ang[2])
    cz = np.cos(ang[2])

    mat[0,0]  = cy*cz
    mat[1,0]  = sx*sy*cz + cx*sz
    mat[2,0]  = -cx*sy*cz + sx*sz
    mat[3,0]  = 0.0
    mat[0,1]  = -cy*sz
    mat[1,1]  = -sx*sy*sz + cx*cz
    mat[2,1]  = cx*sy*sz + sx*cz
    mat[3,1]  = 0.0
    mat[0,2]  = sy
    mat[1,2]  = -sx*cy
    mat[3,2] = cx*cy
    mat[3,2] = 0.0
    mat[0,3] = pos[0]
    mat[1,3] = pos[1]
    mat[2,3] = pos[2]
    mat[3,3] = 1

def Matrix4ToEuler(mat, angInDeg=False):
    '''converts a 4x4 matrix into euler angles'''
    pos = [0,0,0]
    ang = [0,0,0]
    #Calculate Y-axis angle
    if mat[0,0] > 0.0:
        ang[1] = np.arcsin(mat[0,2])
    else:
        ang[1] = np.pi - np.arcsin(mat[0,2])
    C = np.cos(ang[1])
    if np.fabs(C) > 0.005:               # Gimbal lock?
        _trX =  mat[2,2] / C             # No, so get X-axis angle
        _trY =  -mat[1,2] / C
        ang[0] = np.arctan2(_trY, _trX)
        _trX = mat[0,0] / C              # Get Z-axis angle
        _trY = -mat[0,1] / C
        ang[2] = np.arctan2(_trY, _trX)
    else:                                # Gimbal lock has occurred
        ang[0] = 0.0;                    # Set X-axis angle to zero
        _trX = mat[1,1]                  # And calculate Z-axis angle
        _trY = mat[1,0]  
        ang[2] = np.arctan2(_trY, _trX)
    pos[0] = mat[0,3]
    pos[1] = mat[1,3]
    pos[2] = mat[2,3]
    if angInDeg:
        ang = np.rad2deg(ang)
    return (pos, ang)

def toRieglMat(inMat, scale = 100.0):
    '''Converts a left handed 3DTK matrix into a right handed 4x4 matrix'''
    outMat = np.zeros((4,4))
    outMat[1,1] =  inMat[0,0]
    outMat[2,1] = -inMat[1,0]
    outMat[0,1] = -inMat[2,0]
    outMat[3,1] = -inMat[3,0]
    outMat[1,2] = -inMat[0,1]
    outMat[2,2] =  inMat[1,1]
    outMat[0,2] =  inMat[2,1]
    outMat[3,2] =  inMat[3,1]
    outMat[1,0] = -inMat[0,2]
    outMat[2,0] =  inMat[1,2]
    outMat[0,0] =  inMat[2,2]
    outMat[3,0] =  inMat[3,2]
    outMat[1,3] = -inMat[0,3] / scale
    outMat[2,3] =  inMat[1,3] / scale
    outMat[0,3] =  inMat[2,3] / scale
    outMat[3,3] =  inMat[3,3]
    return outMat

def to3DTKMat(inMat, scale = 100.0):
    '''Converts a right handed 4x4 matrix into a left handed 3DTK matrix'''
    outMat = np.zeros((4,4))
    outMat[0,0] =  inMat[1,1]
    outMat[1,0] = -inMat[2,1]
    outMat[2,0] = -inMat[0,1]
    outMat[3,0] = -inMat[3,1]
    outMat[0,1] = -inMat[1,2]
    outMat[1,1] =  inMat[2,2]
    outMat[2,1] =  inMat[0,2] 
    outMat[3,1] =  inMat[3,2]
    outMat[0,2] = -inMat[1,0]
    outMat[1,2] =  inMat[2,0]  
    outMat[2,2] =  inMat[0,0]  
    outMat[3,2] =  inMat[3,0]
    outMat[0,3] = -inMat[1,3] * scale
    outMat[1,3] =  inMat[2,3] * scale
    outMat[2,3] =  inMat[0,3] * scale
    outMat[3,3] =  inMat[3,3]
    return outMat

def toRieglPos(inPos, scale=100.0):
    '''converts a left handed 3DTK point into a right handed point'''
    outPos = np.zeros(3)
    outPos[1] = -inPos[0] / scale
    outPos[2] =  inPos[1] / scale
    outPos[0] =  inPos[2] / scale
    return outPos

def to3DTKPos(inPos, scale=100.0):
    '''converts a right handed point into a left handed 3DTK point'''
    outPos = np.zeros(3)
    outPos[0] = -inPos[1] * scale
    outPos[1] =  inPos[2] * scale
    outPos[2] =  inPos[0] * scale
    return outPos

if __name__ == '__main__':
    baseFilename = os.path.join(os.path.dirname(__file__), scanFolder)
    #get and transform the robot poses:
    T = []
    for i in range(numMidPoses + 2):
        filename = os.path.join(baseFilename, "scan00")
        filename += str(i)
        filename += ".frames"
        with open(filename) as file:
            for line in file:
                pass
            T.append(np.transpose(np.reshape([float(x) for x in line.split()][0:-1], (4,4))))

    robotMidPoses = []
    robotMidPosesEuler = []
    for i in range(numMidPoses):
        robotMidPoses.append(toRieglMat(np.dot(T[i + 1], np.linalg.inv(T[0]))))
        robotMidPosesEuler.append(Matrix4ToEuler(robotMidPoses[-1]))

    robotFinalPose = toRieglMat(np.dot(T[-1], np.linalg.inv(T[0])))
    robotFinalPoseEuler = Matrix4ToEuler(robotFinalPose)

    #get and transform the anchor position:
    posAnchors = []
    for i in range(numTripods):
        filename = os.path.join(baseFilename, "tripod")
        filename += str(i)
        filename += ".3d"
        with open(filename) as file:
            posAnchors.append(toRieglPos(np.dot(T[0], np.block([np.mean([[float(x) for x in line.split()] for line in file], axis=0), 1]))))

    #print poses:
    for i in range(numMidPoses):
        print("MidPose", i, ": ", robotMidPosesEuler[i], sep='')
    print("FinalPose:", robotFinalPoseEuler)
    for i in range(numTripods):
        print("AnchorPos", i, ": ", posAnchors[i], sep='')

    #print top copy to turtlesim_representation scenario.py:
    for i in range(numMidPoses):
        print(robotMidPosesEuler[i][0][0], robotMidPosesEuler[i][0][1], robotMidPosesEuler[i][1][2], sep=', ', end=', ')
    print(robotFinalPoseEuler[0][0], robotFinalPoseEuler[0][1], robotFinalPoseEuler[1][2], sep=', ', end=', ')
    for i in range(numTripods):
        print(posAnchors[i][0], posAnchors[i][1], sep=', ', end=', ')