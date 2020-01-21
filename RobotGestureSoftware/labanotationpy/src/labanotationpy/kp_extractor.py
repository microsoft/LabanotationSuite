#!/usr/bin/env python

import numpy as np
import math
import accessory as ac


def gaussian_pecdec(data, local_max=True):
    mx_pos, mn_pos = 0, 0
    mx, mn = data[0], data[0]

    local_max_peaks = []
    local_min_peaks = []

    delta = 0.1**6
    is_detecting_local_max = False
    
    i = 0
    while i < len(data):

        if data[i] > mx:
            mx_pos = i
            mx = data[i]

        if data[i] < mn:
            mn_pos = i
            mn = data[i]

        if is_detecting_local_max and data[i] < mx-delta:
            local_max_peaks.append(mx_pos)
            is_detecting_local_max = False
            i = mx_pos - 1
            mn = data[mx_pos]
            mn_pos = mx_pos

        elif (not is_detecting_local_max) and data[i] > mn+delta:
            local_min_peaks.append(mn_pos)
            is_detecting_local_max = True
            i = mn_pos - 1
            mx = data[mn_pos]
            mx_pos = mn_pos
        i+=1

    if local_max:
        return local_max_peaks
    else:
        return local_min_peaks


def gaussian_pecdec_extremum(data):
    mx_pos, mn_pos = 0, 0
    mx, mn = data[0], data[0]

    peaks = []

    delta = 0.1**6
    is_detecting_local_max = False
    
    i = 0
    while i < len(data):

        if data[i] > mx:
            mx_pos = i
            mx = data[i]

        if data[i] < mn:
            mn_pos = i
            mn = data[i]

        if is_detecting_local_max and data[i] < mx-delta:
            peaks.append(mx_pos)
            is_detecting_local_max = False
            i = mx_pos - 1
            mn = data[mx_pos]
            mn_pos = mx_pos

        elif (not is_detecting_local_max) and data[i] > mn+delta:
            peaks.append(mn_pos)
            is_detecting_local_max = True
            i = mn_pos - 1
            mx = data[mn_pos]
            mx_pos = mn_pos
        i+=1

    return peaks


def calc_energy(v_w, a_w):
    if  len(v_w.shape) == 2 and len(a_w.shape) == 2 and v_w.shape[1] == 3 and a_w.shape[1] == 3 and v_w.shape[0] == a_w.shape[0]:
        row = v_w.shape[0]
        e = np.zeros((2,row)) # right/left, velocity

        for i in range(row):
            e[0,i] = np.sqrt((a_w[i][0]**2+a_w[i][1]**2+a_w[i][2]**2)/3.0) #a_w
            e[1,i] = np.sqrt((v_w[i][0]**2+v_w[i][1]**2+v_w[i][2]**2)/3.0) #v_w

        # Do the normalization then add them toghether
        for i in range(len(e)):
            e[i] = ac.norm(e[i])
            
        total_e = np.zeros((row))
        for i in range(row):
            total_e[i] = e[1,i]
        return total_e
    
    else:
        print("Energy function input data error!")
        return
