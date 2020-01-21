# --------------------------------------------------------------------------------------------
# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.
# --------------------------------------------------------------------------------------------

import os,math,copy
import numpy as np

import settings

# a joint point, 'ts' stands for tracking status
jType = np.dtype({'names':['x', 'y', 'z','ts'],'formats':[float,float,float,int]})

# a body
bType = np.dtype({'names':[ 'timeS',                # milliseconds
                            'filled',               # filled gap
                            'spineB', 'spineM',     # meter
                            'neck', 'head',
                            'shoulderL', 'elbowL', 'wristL', 'handL', # tracked=2, inferred=1, nottracked=0
                            'shoulderR', 'elbowR', 'wristR', 'handR',
                            'hipL', 'kneeL', 'ankleL', 'footL',
                            'hipR', 'kneeR', 'ankleR', 'footR',
                            'spineS', 'handTL', 'thumbL', 'handTR', 'thumbR'],
                'formats':[ long,
                            bool,
                            jType, jType,
                            jType, jType,
                            jType, jType, jType, jType,
                            jType, jType, jType, jType,
                            jType, jType, jType, jType,
                            jType, jType, jType, jType,
                            jType, jType, jType, jType, jType]})


#------------------------------------------------------------------------------
# load kinect data file an return as a bType array
#
def loadKinectDataFile(filePath, fFillGap = False):
    if not os.path.isabs(filePath):
        print('input file ' + os.path.basename(filePath) + ' does not exist.')
        exit()

    f = open(filePath)

    kinectData = []
    idx = 0
    currentTime = 0
    startTime = 0
    lastTime = 0

    line = f.readline()
    while line != '':
        temp = line.split(',')
        if len(temp) < 1+25*3:
            break

        currentTime = int(temp[0])

        tempBody = np.zeros(1, dtype=bType)
        tempBody['filled'] = False
        if (idx == 0):
            tempBody['timeS'] = 1
            startTime = currentTime
        else:
            # from kinect data
            if currentTime > 10e7:
                cnt = ((currentTime - lastTime) / 10000) / 30
                if (cnt < 1):
                    cnt = 1
                tempBody['timeS'] = kinectData[-1][0][0] + cnt*33
            # yamada data
            else:
                tempBody['timeS'] = (currentTime - startTime)

        # get joints
        for j in range(0, 25):
            tempPoint = np.zeros(1, dtype=jType)
            tempPoint['x'] =  float(temp[1+j*4])
            tempPoint['y'] =  float(temp[2+j*4])
            tempPoint['z'] =  float(temp[3+j*4])
            tempPoint['ts'] = int(temp[4+j*4])
            tempBody[0][j+2] = tempPoint

        # fill time gap when needed
        if ((fFillGap == True) and (idx > 0)):
            timeGap = (currentTime - lastTime) / 10000
            if (timeGap > 40):
                cnt = timeGap/30
                if (settings.fVerbose):
                    print('index ' + str(i) + ' is ' + str(timeGap) + 'ms')
                    print('adding ' + str(cnt) + ' frame')

                refPoseA = kinectData[-1][0]
                refPoseB = tempBody[0]

                for j in range(1, cnt):
                    extraBody = np.zeros(1, dtype=bType)
                    # add time first
                    extraBody['timeS'] = 1 + 33*idx
                    extraBody['filled'] = True
                    # then add joints
                    # do a linear interpolation between two poses (refPoseB and refPoseA). If error margins are 
                    # important, replace this interpolation with a joint corrected approach
                    for k in range(2, 27):
                        xGap = (refPoseB[k][0] - refPoseA[k][0])
                        yGap = (refPoseB[k][1] - refPoseA[k][1])
                        zGap = (refPoseB[k][2] - refPoseA[k][2])

                        extraPoint = np.zeros(1,dtype=jType)
                        extraPoint['x'] =  refPoseA[k][0] + (xGap * float(j) / float(cnt))
                        extraPoint['y'] =  refPoseA[k][1] + (yGap * float(j) / float(cnt))
                        extraPoint['z'] =  refPoseA[k][2] + (zGap * float(j) / float(cnt))
                        extraPoint['ts'] = 0
                        extraBody[0][k] = extraPoint

                    kinectData.append(extraBody)
                    idx += 1
            elif timeGap < 30:
                pass
                #print str(i) + ' is ' + 'smaller than 30ms! ' + str(timeGap) + 'ms'

        kinectData.append(tempBody)

        lastTime = currentTime
        idx += 1
        line = f.readline()

    f.close()

    return kinectData

