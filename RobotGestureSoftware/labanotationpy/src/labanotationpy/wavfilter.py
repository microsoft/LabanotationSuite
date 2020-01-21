#!/usr/bin/env python

import numpy as np
import math


def gaussFilter(windowSize=5, sig=3):
    if windowSize % 2 == 1:
        #odd
        gauss = [i-(windowSize//2) for i in range(windowSize)]
    else:
        #even
        gauss = []
        for i in range(windowSize):
            if i < windowSize/2:
                temp = i - windowSize/2 + 1
            else:
                temp = i - windowSize/2
            gauss.append(float(temp))

    #f(x) = a*e^(-(x-b)^2/(2*c^2))
    for i in range(windowSize):
        temp = - (gauss[i])**2/(2.0*sig**2)
        gauss[i] = math.e**temp
    n = sum(gauss)
    for i in range(windowSize):
        gauss[i] = gauss[i]/n
    return np.array(gauss)


def calcFilter(raw, gauss):
    tempOut = []
    row = raw.shape[0]
    if len(raw.shape) > 1:
        col = raw.shape[1]
        tempArray = (raw.transpose()).tolist()
        glist = gauss.tolist()
        for i in range(0, col):
            temp = [tempArray[i][0]]*(len(glist)/2) + tempArray[i] + [tempArray[i][-1]]*(len(glist)/2)
            for j in range(0, len(temp)-len(glist)+1):
                tempOut.append(np.dot(temp[j:j+len(glist)],glist))
        filtered = np.array(tempOut)
        filtered = filtered.reshape(col,row)
        return filtered.transpose()
    else:
        tempArray = raw.tolist()
        glist = gauss.tolist()
        temp = [tempArray[0]]*(len(glist)/2) + tempArray + [tempArray[-1]]*(len(glist)/2)
        for i in range(0, len(temp)-len(glist)+1):
            tempOut.append(np.dot(temp[i:i+len(glist)],glist))
        filtered = np.array(tempOut)
        return filtered
