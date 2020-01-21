# --------------------------------------------------------------------------------------------
# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.
# --------------------------------------------------------------------------------------------

import math
import numpy as np

import accessory as ac

#------------------------------------------------------------------------------
# build 1 dimensional gaussian filter
#   f->f: output gaussin array, length is "n".This will be a return value in python.
#   n->n: m=101
#   s->s: sigma
#   N->n_up: EnergyDattKP.BodyDetection, usually when it reached 101 then start to process (>=101).
#
def build_gaussian_filter_1d(n, s, n_up):
    f = np.zeros((n))
    if (s <= 0):
        for i in range(0, n):
            f[i] = 1.0
        f[int(round((n-1)/2.0))] = 1
        return f
    x = np.zeros((n))
    sumup = 0.0
    for i in range(0,n):
        x[i] = (i-(n-1)/2.0)/(n_up-1)
        f[i] = math.e**(-1*x[i]**2/(s**2*2))
        sumup += f[i]
    f = f/sumup

    return f

#------------------------------------------------------------------------------
#
def convolution():
    pass

#------------------------------------------------------------------------------
#
def performance_convolution():
    pass

#------------------------------------------------------------------------------
# data : DXYZ. Input data, energy array
#
def gaussian_pecdec(data):
    mx_pos = 0
    mn_pos = 0
    mx = data[0]
    mn = data[0]

    emi_peaks = []
    absop_peaks = []

    delta = 0.1**6
    emi_first = 0

    is_detecting_emi = emi_first
    i = 0
    while i < len(data):
        if data[i] > mx:
            mx_pos = i
            mx = data[i]
        if data[i] < mn:
            mn_pos = i
            mn = data[i]
        # the curve is going down
        if is_detecting_emi != 0 and data[i] < mx-delta:
            emi_peaks.append(mx_pos)
            is_detecting_emi = 0
            i = mx_pos -1
            mn = data[mx_pos]
            mn_pos = mx_pos
        elif is_detecting_emi==0 and data[i] > mn+delta:
            absop_peaks.append(mn_pos)
            is_detecting_emi = 1
            i = mn_pos - 1
            mx = data[mn_pos]
            mx_pos = mn_pos
        i+=1

    return emi_peaks

#------------------------------------------------------------------------------
# calculate the energy function by given matrix (velocity in 2d, acceleration in 2d),
# output a energy matrix (1d).
# 
def energy_function(v_l, a_l, v_r, a_r):
    if  len(v_l.shape) == 2 and len(a_l.shape) == 2\
    and len(v_r.shape) == 2 and len(a_r.shape) == 2\
    and v_l.shape[1] ==3 and a_l.shape[1] ==3\
    and v_r.shape[1]==3 and a_r.shape[1]==3\
    and v_l.shape[0]==a_l.shape[0]\
    and a_l.shape[0]==v_r.shape[0]\
    and v_r.shape[0]==a_r.shape[0]:
        row = v_l.shape[0]
        e = np.zeros((row))
        
        # normalize each axis then get value first
        v_l = ac.norm(v_l)
        v_r = ac.norm(v_r)
        a_l = ac.norm(a_l)
        a_r = ac.norm(a_r)
        
        for i in range(0,row):
            l = np.sqrt((a_l[i][0]**2+a_l[i][1]**2+a_l[i][2]**2)/3.0)\
                -np.sqrt((v_l[i][0]**2+v_l[i][1]**2+v_l[i][2]**2)/3.0)
            r = np.sqrt((a_r[i][0]**2+a_r[i][1]**2+a_r[i][2]**2)/3.0)\
                -np.sqrt((v_r[i][0]**2+v_r[i][1]**2+v_r[i][2]**2)/3.0)
            e[i] = r+l

        top = max(e)
        btm = min(e)
        return (e-btm)/(top-btm)
    else:
        print("Energy function input data error!")
        return

#------------------------------------------------------------------------------
# update energy function for IJCV paper 2018
#
def energy_function_ijcv(v_l, a_l, v_r, a_r):
    if  len(v_l.shape) == 2 and len(a_l.shape) == 2\
    and len(v_r.shape) == 2 and len(a_r.shape) == 2\
    and v_l.shape[1] ==3 and a_l.shape[1] ==3\
    and v_r.shape[1]==3 and a_r.shape[1]==3\
    and v_l.shape[0]==a_l.shape[0]\
    and a_l.shape[0]==v_r.shape[0]\
    and v_r.shape[0]==a_r.shape[0]:
        row = v_l.shape[0]
        e = np.zeros((4,row)) # right/left, velocity/acceleration

        for i in range(row):
            e[0,i] = np.sqrt((a_l[i][0]**2+a_l[i][1]**2+a_l[i][2]**2)/3.0) # a_l
            e[1,i] = np.sqrt((v_l[i][0]**2+v_l[i][1]**2+v_l[i][2]**2)/3.0) # v_l
            e[2,i] = np.sqrt((a_r[i][0]**2+a_r[i][1]**2+a_r[i][2]**2)/3.0) # a_r
            e[3,i] = np.sqrt((v_r[i][0]**2+v_r[i][1]**2+v_r[i][2]**2)/3.0) # a_r

        # do the normalization then add them toghether
        for i in range(4):
            e[i] = ac.norm(e[i])
            
        total_e = np.zeros((row))
        for i in range(row):
            total_e[i] = e[0,i] + e[2,i] - e[1,i] - e[3,i]

        return total_e
    else:
        print("Energy function input data error!")
        return
