# --------------------------------------------------------------------------------------------
# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.
# --------------------------------------------------------------------------------------------

import math
import numpy as np

#------------------------------------------------------------------------------
# Normalize a 1 dimension vector
#
def norm1d(a):
    if len(a.shape) != 1:
        return -1
    else:
        l = a.shape[0]
        s = 0 #sum
        for i in range(0,l):
            s+=a[i]**2
        s = math.sqrt(s)
        v = np.zeros(l)
        for i in range(0,l):
            v[i] = a[i]/s
        return v
    
#------------------------------------------------------------------------------
# Converting a vector from Cartesian coordianate to spherical
# theta:0~180 (zenith), phi: 0~180 for left, 0~-180 for right (azimuth)
#
def to_sphere(a):
    x = a[0]
    y = a[1]
    z = a[2]
    r = math.sqrt(x**2+y**2+z**2)
    if r == 0:
        return (0.0,0.0,0.0)
    theta = math.degrees(math.acos(z/r))
    # phi is from -180~+180
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


def calculate_base_rotation(joint):
    shL = np.zeros(3)
    shR = np.zeros(3)
    spM = np.zeros(3)

    shL[0] = joint[0]['shoulderL']['x']
    shL[1] = joint[0]['shoulderL']['y']
    shL[2] = joint[0]['shoulderL']['z']
    shR[0] = joint[0]['shoulderR']['x']
    shR[1] = joint[0]['shoulderR']['y']
    shR[2] = joint[0]['shoulderR']['z']

    spM[0] = joint[0]['spineM']['x']
    spM[1] = joint[0]['spineM']['y']
    spM[2] = joint[0]['spineM']['z']

    # convert kinect space to spherical coordinate
    # 1. normal vector of plane defined by shoulderR, shoulderL and spineM
    sh = np.zeros((3,3))
    v1 = shL-shR
    v2 = spM-shR
    sh[0] = np.cross(v2,v1)#x axis
    sh[1] = v1#y axis
    sh[2] = np.cross(sh[0],sh[1])#z axis
    nv = np.zeros((3,3))
    nv[0] = norm1d(sh[0])
    nv[1] = norm1d(sh[1])
    nv[2] = norm1d(sh[2])
    # 2. generate the rotation matrix for
    # converting point from kinect space to euculid space, then sphereical
    base_rotation = np.transpose(nv)
    return base_rotation


#------------------------------------------------------------------------------
# Transform origin from kinect-base to shoulder-base, 
# convert position information to angle/direction+level
# Replace LabaProcessor::(FindDriectionXOZ, FindLevelYOZ, FindLevelXOY)
# 
def raw2sphere(joint, base_rotation=None):
    shL = np.zeros(3)
    shR = np.zeros(3)
    elbowR = np.zeros(3)
    elbowL = np.zeros(3)
    wristR = np.zeros(3)
    wristL = np.zeros(3)

    shL[0] = joint[0]['shoulderL']['x']
    shL[1] = joint[0]['shoulderL']['y']
    shL[2] = joint[0]['shoulderL']['z']
    shR[0] = joint[0]['shoulderR']['x']
    shR[1] = joint[0]['shoulderR']['y']
    shR[2] = joint[0]['shoulderR']['z']

    elbowR[0] = joint[0]['elbowR']['x']-shR[0]
    elbowR[1] = joint[0]['elbowR']['y']-shR[1]
    elbowR[2] = joint[0]['elbowR']['z']-shR[2]
    elbowL[0] = joint[0]['elbowL']['x']-shL[0]
    elbowL[1] = joint[0]['elbowL']['y']-shL[1]
    elbowL[2] = joint[0]['elbowL']['z']-shL[2]
    
    wristR[0] = joint[0]['wristR']['x']-joint[0]['elbowR']['x']
    wristR[1] = joint[0]['wristR']['y']-joint[0]['elbowR']['y']
    wristR[2] = joint[0]['wristR']['z']-joint[0]['elbowR']['z']
    wristL[0] = joint[0]['wristL']['x']-joint[0]['elbowL']['x']
    wristL[1] = joint[0]['wristL']['y']-joint[0]['elbowL']['y']
    wristL[2] = joint[0]['wristL']['z']-joint[0]['elbowL']['z']

    if base_rotation is None:
        conv = calculate_base_rotation(joint)
    else:
        conv = base_rotation

    elRdeg = to_sphere(np.dot(conv.T, elbowR))
    elLdeg = to_sphere(np.dot(conv.T, elbowL))
    wrRdeg = to_sphere(np.dot(conv.T, wristR))
    wrLdeg = to_sphere(np.dot(conv.T, wristL))

    return (elRdeg,elLdeg,wrRdeg,wrLdeg)

#------------------------------------------------------------------------------
# replace LabaProcessor::CoordinateToLabanotation, FindDirectionsHML, FindDirectionsFSB
#
# Direction:
# forward--'f', rightforward--'rf', right--'r',rightbackward--'rb'
# backward--'b', leftbackward--'lb',left--'l',leftforward--'lf'
#
# Height:
# place high--'ph', high--'h', middle/normal--'m', low--'l', place low--'pl'
#
def coordinate2laban(theta, phi):
    laban = ['Normal', 'Forward']
    
    #find direction, phi, (-180,180]
    #forward
    if (phi <= 22.5 and phi >= 0) or (phi < 0 and phi > -22.5):
        laban[0] = 'Forward'
    elif (phi <= 67.5 and phi > 22.5):
        laban[0] = 'Left Forward'
    elif (phi <= 112.5 and phi > 67.5):
        laban[0] = 'Left'
    elif (phi <= 157.5 and phi > 112.5):
        laban[0] = 'Left Backward'
    elif (phi <= -157.5 and phi > -180) or (phi <= 180 and phi > 157.5):
        laban[0] = 'Backward'
    elif (phi <= -112.5 and phi > -157.5):
        laban[0] = 'Right Backward'
    elif (phi <= -67.5 and phi > -112.5):
        laban[0] = 'Right'
    else:
        laban[0] = 'Right Forward'
    
    # find height, theta, [0,180]
    # place high
    if theta < 22.5:
        laban=['Place','High']
    # high
    elif theta < 67.5:
        laban[1] = 'High'
    # normal/mid
    elif theta < 112.5:
        laban[1] = 'Normal'
    # low
    elif theta < 157.5:
        laban[1] = 'Low'
    # place low
    else:
        laban = ['Place','Low']

    return laban

#------------------------------------------------------------------------------
#
def LabanKeyframeToScript(idx, time, dur, laban_score):
    strScript = ""

    strScript += '#' + str(idx) + '\n'
    strScript += 'Start Time:'+ str(time) +'\nDuration:' + str(dur) + '\nHead:Forward:Normal\n'
    strScript += 'Right Elbow:' + laban_score[0][0] + ':' + laban_score[0][1] + '\n'
    strScript += 'Right Wrist:' + laban_score[1][0] + ':' + laban_score[1][1] + '\n'
    strScript += 'Left Elbow:'  + laban_score[2][0] + ':' + laban_score[2][1] + '\n'
    strScript += 'Left Wrist:'  + laban_score[3][0] + ':' + laban_score[3][1] + '\n'
    strScript += 'Rotation:ToLeft:0.0\n'

    return strScript

#------------------------------------------------------------------------------
#
def toScript(timeS, all_laban):
    if (all_laban == None):
        return ""

    strScript = ""
    cnt = len(all_laban)
    for j in range(cnt):
        if j == 0:
            time = 1
        else:
            time = int(timeS[j])

        if j == (cnt - 1):
            dur = '-1'
        else:
            dur = '1'

        strScript += LabanKeyframeToScript(j, time, dur, all_laban[j])

    return strScript

