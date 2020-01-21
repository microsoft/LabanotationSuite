#!/usr/bin/env python

import numpy as np


'''
Calculate the velocity from given position matrix (2d or 1d) and time matrix,
output a velocity matrix (2d or 1d). The assumption before calculating is v0 = 0.
col: usually 2 or 1.
row: should be same as time.
'''
def vel(time, pos):

    row = pos.shape[0]
    if len(pos.shape) > 1:
        col = pos.shape[1]
        v = np.zeros((row,col))
        for i in range(1,row):
            for j in range(0,col):
                v[i][j] = (pos[i][j]-pos[i-1][j])/(time[i]-time[i-1])
        for j in range(0,col):
            v[0][j] = v[1][j]
    else:
        v = np.zeros((row))
        for i in range(1,row):
            v[i] = (pos[i]-pos[i-1])/(time[i]-time[i-1])
        v[0] = v[1]

    return v


'''
Calculate the acceleration from given velocity matrix (2d or 1d) and time matrix,
output a acceleration matrix (2d or 1d). The assumption before calculating is an = 0.
col: usually 2 or 1.
row: should be same as time.
'''
def acc(time, vel):

    row = vel.shape[0]
    if len(vel.shape) > 1:
        col = vel.shape[1]
        a = np.zeros((row,col))
        for i in range(1,row):
            for j in range(0,col):
                a[i][j] = (vel[i][j]-vel[i-1][j])/(time[i]-time[i-1])
        for j in range(0,col):
            a[0][j] = a[1][j]
    else:
        a = np.zeros((row))
        for i in range(1,row):
            a[i] = (vel[i]-vel[i-1])/(time[i]-time[i-1])
        a[0] = a[1]

    return a


'''
Calculate the value from a vector.
'''
def ndto1d(vec):

    row = vec.shape[0]
    val = []
    if len(vec.shape) > 1:
        col = vec.shape[1]
        for i in range(0,row):
            temp = 0
            for j in range(0,col):
                temp += vec[i][j]**2
            temp = np.sqrt(temp)
            val.append(temp)
    else:
        col = 1
        for i in range(0,row):
            val.append(np.abs(vec[i]))

    return np.array(val)

'''
Normalizing a set of 2d or 1d data.
''' 
def norm(temp):
    
    row = temp.shape[0]
    if len(temp.shape) > 1:
        col = temp.shape[1]
        a = np.zeros((row,col))
        for i in range(0,col):
            top = max(temp[:,i:i+1])
            btm = min(temp[:,i:i+1])
            if (top-btm) == 0:
                a[:,i:i+1] = 0
            else:
                a[:,i:i+1] = (temp[:,i:i+1]-btm)/(top-btm)
        return a
            
    else:
        top = max(temp)
        btm = min(temp)
        if top-btm==0:
            return np.array(len(temp))
        else:
            return np.array([(temp[i]-btm)/(top-btm) for i in range(0,len(temp))])


#split one motion into different set according to the Labanotation
#[[start_0,end_0],[start_1,end_1],...]
def split(laban):
    cnt = len(laban)
    #ptr always pointed to the 
    ptr = 1
    start = 0
    laban_sect = []
    while ptr < cnt and start < cnt:
        while laban[ptr] == laban[start] and ptr < cnt-1:
            ptr += 1
        # (x_start, y_start), x_width, y_width, alpha
        laban_sect.append([start,ptr-1])
        start = ptr
        ptr += 1
    return laban_sect


def search_forward(laban,pos):
    ptr = pos-1
    while laban[ptr] == laban[pos]:
        ptr -= 1
    ptr += 1
    return ptr


def der(y):
    n = len(y)
    y_der = []

    for i in range(n-1):
        y_der.append(y[i+1]-y[i])
    y_der.append(y_der[-1])
    y_der = np.array(y_der)
    
    return y_der


def inflection(y, thres_1=0.1**3, thres_2=0.1**3):
    n = len(y)
    y_d = der(y)

    y_dd = der(der(y))
    y_dd = 10.0*y_dd/(max(y_dd)-min(y_dd))

    y_ddd = der(der(der(y)))
    y_ddd = 10.0*y_ddd/(max(y_ddd)-min(y_ddd))

    infl = [0]
    #F_xx = 0 & F_xxx != 0
    for i in range(len(y_dd)-1):
        if (y_dd[i] < 0 and y_dd[i+1] > 0) or (y_dd[i] > 0 and y_dd[i+1] < 0):
            if abs(y_ddd[i]) > 0.1:
                if abs(y_dd[i] < y_dd[i+1]):
                    infl.append(i)
                else:
                    infl.append(i+1)
    infl.append(len(y_dd)-1)
    return infl
