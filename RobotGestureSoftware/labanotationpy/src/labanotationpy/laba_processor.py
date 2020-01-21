#!/usr/bin/env python

import sys
import math
import numpy as np
from collections import OrderedDict


'''
Normalize a 1 dimension vector
'''
def norm1d(a):
    if len(a.shape) != 1:
        return -1
    else:
        l = a.shape[0]
        s = 0#sum
        for i in range(0,l):
            s+=a[i]**2
        s = math.sqrt(s)
        v = np.zeros(l)
        for i in range(0,l):
            v[i] = a[i]/s
        return v


'''
Converting a vector from Cartesian coordianate to spherical
theta: 0~180 (zenith), phi: 0~180 for left, 0~-180 for right (azimuth)
'''
def to_sphere(a):
    x = a[0]
    y = a[1]
    z = a[2]
    r = math.sqrt(x**2+y**2+z**2)
    if r == 0:
        return (0.0,0.0,0.0)
    theta = math.degrees(math.acos(z/r))
    #phi is from -180~+180
    if x != 0.0:
        phi = math.degrees(math.atan(y/x))
    else:
        if y > 0:
            phi = 90
        else:
            phi = -90
    if x < 0 and y > 0:
        phi = 180+phi
    elif x < 0 and y < 0:
        phi = -180+phi
    else:
        phi = phi
    return (r, theta, phi)


'''
Transform origin from kinect-base to shoulder-base, 
convert position information to angle/direction+level
Replace LabaProcessor::(FindDriectionXOZ, FindLevelYOZ, FindLevelXOY)
'''
def raw2sphere(jd):

    shL, shR, spM = np.zeros(3), np.zeros(3), np.zeros(3)
    elbowR, elbowL, wristR, wristL = np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3)
    handR, handL, palmR, palmL = np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3)
    wholeTorso = np.zeros(3)

    shL = np.array([jd[0]['shoulderL']['x'], jd[0]['shoulderL']['y'], jd[0]['shoulderL']['z']])
    shR = np.array([jd[0]['shoulderR']['x'], jd[0]['shoulderR']['y'], jd[0]['shoulderR']['z']])
    spM = np.array([jd[0]['spineM']['x'], jd[0]['spineM']['y'], jd[0]['spineM']['z']])

    elbowL = np.array([jd[0]['elbowL']['x']-shL[0], jd[0]['elbowL']['y']-shL[1], jd[0]['elbowL']['z']-shL[2]])
    elbowR = np.array([jd[0]['elbowR']['x']-shR[0], jd[0]['elbowR']['y']-shR[1], jd[0]['elbowR']['z']-shR[2]])

    wristL = np.array([jd[0]['wristL']['x']-jd[0]['elbowL']['x'], jd[0]['wristL']['y']-jd[0]['elbowL']['y'], jd[0]['wristL']['z']-jd[0]['elbowL']['z']])
    wristR = np.array([jd[0]['wristR']['x']-jd[0]['elbowR']['x'], jd[0]['wristR']['y']-jd[0]['elbowR']['y'], jd[0]['wristR']['z']-jd[0]['elbowR']['z']]) 

    handL = np.array([jd[0]['middleMPL']['x']-jd[0]['handL']['x'], jd[0]['middleMPL']['y']-jd[0]['handL']['y'], jd[0]['middleMPL']['z']-jd[0]['handL']['z']])
    handR = np.array([jd[0]['middleMPR']['x']-jd[0]['handR']['x'], jd[0]['middleMPR']['y']-jd[0]['handR']['y'], jd[0]['middleMPR']['z']-jd[0]['handR']['z']])

    wrist2pinkyMPL = np.array([jd[0]['pinkyMPL']['x']-jd[0]['handL']['x'], jd[0]['pinkyMPL']['y']-jd[0]['handL']['y'], jd[0]['pinkyMPL']['z']-jd[0]['handL']['z']])
    wrist2pinkyMPR = np.array([jd[0]['pinkyMPR']['x']-jd[0]['handR']['x'], jd[0]['pinkyMPR']['y']-jd[0]['handR']['y'], jd[0]['pinkyMPR']['z']-jd[0]['handR']['z']])

    palmR = np.cross(handR, wrist2pinkyMPR)
    palmL = np.cross(wrist2pinkyMPL, handL)

    wholeTorso = np.array([ \
        jd[0]['spineS']['x']-jd[0]['spineB']['x'], \
        jd[0]['spineS']['y']-jd[0]['spineB']['y'], \
        jd[0]['spineS']['z']-jd[0]['spineB']['z']])

    # convert kinect space to spherical coordinate
    # 1. normal vector of plane defined by shoulderR, shoulderL and spineM
    sh = np.zeros((3,3))
    
    v1 = shL-shR
    v2 = spM-shR

    sh[0] = np.cross(v2,v1) # x axis
    sh[1] = v1 # y axis
    sh[2] = np.cross(sh[0],sh[1]) # z axis
    
    nv = np.zeros((3,3))
    
    # unit vector
    nv[0] = norm1d(sh[0])
    nv[1] = norm1d(sh[1])
    nv[2] = norm1d(sh[2])

    # 2. generate the rotation matrix for
    # converting point from kinect space to euculid space, then sphereical
    conv = np.linalg.inv(np.transpose(nv))

    elRdeg = to_sphere(np.dot(conv,elbowR))
    elLdeg = to_sphere(np.dot(conv,elbowL))
    wrRdeg = to_sphere(np.dot(conv,wristR))
    wrLdeg = to_sphere(np.dot(conv,wristL))
    hRdeg = to_sphere(np.dot(conv,handR))
    hLdeg = to_sphere(np.dot(conv,handL))
    pRdeg = to_sphere(np.dot(conv,palmR))
    pLdeg = to_sphere(np.dot(conv,palmL))
    wTdeg = to_sphere(np.dot(conv,wholeTorso))

    return (elRdeg,elLdeg,wrRdeg,wrLdeg,hRdeg,hLdeg,pRdeg,pLdeg,wTdeg)


'''
replace LabaProcessor::CoordinateToLabanotation, FindDirectionsHML, FindDirectionsFSB

Direction:
forward, rightforward, right, rightbackward, 
backward, leftbackward, left, leftforward, place

Level:
high, middle, low
'''
def coordinate2laban(theta, phi):
    laban = OrderedDict({"Direction": 'forward', 'Level': 'middle'})
    
    # find direction, phi, (-180,180]
    # forward
    if (phi <= 22.5 and phi >= 0) or (phi < 0 and phi > -22.5):
        laban['Direction'] = 'forward'
    elif (phi <= 67.5 and phi > 22.5):
        laban['Direction'] = 'leftForward'
    elif (phi <= 112.5 and phi > 67.5):
        laban['Direction'] = 'left'
    elif (phi <= 157.5 and phi > 112.5):
        laban['Direction'] = 'leftBackward'
    elif (phi <= -157.5 and phi > -180) or (phi <= 180 and phi > 157.5):
        laban['Direction'] = 'backward'
    elif (phi <= -112.5 and phi > -157.5):
        laban['Direction'] = 'rightBackward'
    elif (phi <= -67.5 and phi > -112.5):
        laban['Direction'] = 'right'
    else:
        laban['Direction'] = 'rightForward'
    
    # find height, theta, [0,180]
    # place high
    if theta < 22.5:
        laban = OrderedDict({"Direction": 'place', 'Level': 'high'})
    # high
    elif theta < 67.5:
        laban['Level'] = 'high'
    # normal/middle
    elif theta < 112.5:
        laban['Level'] = 'middle'
    # low
    elif theta < 157.5:
        laban['Level'] = 'low'
    # place low
    else:
        laban = OrderedDict({"Direction": 'place', 'Level': 'low'})

    return laban


def calc_shoulder_vec(jd):
    return np.array([jd[0]['shoulderR']['x']-jd[0]['shoulderL']['x'], \
        jd[0]['shoulderR']['y']-jd[0]['shoulderL']['y'], \
        jd[0]['shoulderR']['z']-jd[0]['shoulderL']['z']])


def addTurnSign(laban, init_pose, jd):
    laban['Sign'] = 'turnNormal'
    
    # calculate the orientation (x-z plane)
    init_rot_angle = math.degrees(math.atan2(init_pose[0], init_pose[2]))
    sh_vec = calc_shoulder_vec(jd)
    rot_angle = math.degrees(math.atan2(sh_vec[0], sh_vec[2])) - init_rot_angle

    if rot_angle > -22.5 and rot_angle < 22.5: 
        laban['Sign'] = 'turnNormal'
    elif rot_angle >= 22.5 and rot_angle < 67.5: 
        laban['Sign'] = 'turn1/8left'
    elif rot_angle >= 67.5 and rot_angle < 112.5: 
        laban['Sign'] = 'turn1/4left'
    elif rot_angle >= 112.5 and rot_angle < 157.5: 
        laban['Sign'] = 'turn3/8left'
    elif rot_angle >= 157.5:
        laban['Sign'] = 'turn1/2left'
    elif rot_angle <= -22.5 and rot_angle > -67.5: 
        laban['Sign'] = 'turn1/8right'
    elif rot_angle <= -67.5 and rot_angle > -112.5: 
        laban['Sign'] = 'turn1/4right'
    elif rot_angle <= -112.5 and rot_angle > -157.5: 
        laban['Sign'] = 'turn3/8right'
    elif rot_angle <= -157.5:
        laban['Sign'] = 'turn1/2right'

    return laban


def rotMat(p):
    px, py, pz = p[0], p[1], p[2]

    Rx = np.array([[1, 0, 0],
                   [0, np.cos(px), np.sin(px)],
                   [0, -np.sin(px), np.cos(px)]])
    Ry = np.array([[np.cos(py), 0, -np.sin(py)],
                   [0, 1, 0],
                   [np.sin(py), 0, np.cos(py)]])
    Rz = np.array([[np.cos(pz), np.sin(pz), 0],
                   [-np.sin(pz), np.cos(pz), 0],
                   [0, 0, 1]])
    return Rz.dot(Ry).dot(Rx)


def neuron2ROS(offset, pos0, pos, orient):
    # calculate orient_ from the initial position of the human to base frame
    unit_y = np.array([0,1,0])
    vec_foots = np.array([pos0['LeftFoot.X']-pos0['RightFoot.X'], pos0['LeftFoot.Y']-pos0['RightFoot.Y'], pos0['LeftFoot.Z']-pos0['RightFoot.Z']])
    FoB = np.cross(vec_foots, unit_y)  # front of body

    if FoB[2] < 0:
        orient_ = 180-orient
    else:
        orient_ = orient

    # return the human to base frame, use orient_ [deg]
    pos = np.dot(rotMat([0,orient_,0]),pos-offset)

    # convert into the ROS frame with unit conversion from [cm] to [m]
    pos = [pos[2]*0.01, pos[0]*0.01, pos[1]*0.01]

    return pos


def get_positionROSframe(allaban, pos0, pos, rot0):
    # calculate the offset
    offset = np.array([(pos0['LeftFoot.X']+pos0['RightFoot.X'])/2.0, (pos0['LeftFoot.Y']+pos0['RightFoot.Y'])/2.0, (pos0['LeftFoot.Z']+pos0['RightFoot.Z'])/2.0])

    for jd in allaban.keys():
        if jd=='RightElbow':
            allaban[jd]['Position'] = neuron2ROS(offset, pos0, np.array([pos['RightForeArm.X'],pos['RightForeArm.Y'],pos['RightForeArm.Z']]), rot0['Hips.Y'])
        elif jd=='RightWrist':
            allaban[jd]['Position'] = neuron2ROS(offset, pos0, np.array([pos['RightHand.X'],pos['RightHand.Y'],pos['RightHand.Z']]), rot0['Hips.Y'])
        elif jd=='RightHand':
            allaban[jd]['Position'] = neuron2ROS(offset, pos0, np.array([pos['RightInHandMiddle.X'],pos['RightInHandMiddle.Y'],pos['RightInHandMiddle.Z']]), rot0['Hips.Y'])
        elif jd=='LeftElbow':
            allaban[jd]['Position'] = neuron2ROS(offset, pos0, np.array([pos['LeftForeArm.X'],pos['LeftForeArm.Y'],pos['LeftForeArm.Z']]), rot0['Hips.Y'])
        elif jd=='LeftWrist':
            allaban[jd]['Position'] = neuron2ROS(offset, pos0, np.array([pos['LeftHand.X'],pos['LeftHand.Y'],pos['LeftHand.Z']]), rot0['Hips.Y'])
        elif jd=='LeftHand':
            allaban[jd]['Position'] = neuron2ROS(offset, pos0, np.array([pos['LeftInHandMiddle.X'],pos['LeftInHandMiddle.Y'],pos['LeftInHandMiddle.Z']]), rot0['Hips.Y'])
        elif jd=='WholeTorso':
            allaban[jd]['Position'] = neuron2ROS(offset, pos0, np.array([pos['Hips.X'],pos['Hips.Y'],pos['Hips.Z']]), rot0['Hips.Y'])
        else:
            pass # no need to implement for palm

    return allaban
