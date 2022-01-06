# --------------------------------------------------------------------------------------------
# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.
# --------------------------------------------------------------------------------------------

import matplotlib.pyplot as pl
import numpy as np

import accessory as ac
import wavfilter as wf

#------------------------------------------------------------------------------
#
def stats(peaks, cnt, fig):
    data = sum(peaks, [])
    data.sort()
    cnter = np.zeros(cnt)
    for i in range(len(data)):
        cnter[data[i]] += 1

    #81 or 101
    gauss = wf.gaussFilter(101,10)
    cnter = wf.calcFilter(cnter, gauss)

    peaks, _, _ =peak_dect(cnter, y_thres = 0)

    fig.plot(cnter, '+', markersize=7, markevery=peaks)
    print('honnma-kaina')

    fig.plot(cnter)
    fig.set_xlim((0,cnt))

#------------------------------------------------------------------------------
#
def check_status(angle):
    if abs(angle) <= 22.5:
        return 0
    if angle > 22.5 and angle <= 67.5:
        return 1
    if angle > 67.5 and angle <= 112.5:
        return 2
    if angle > 112.5 and angle < 157.5:
        return 3
    if abs(angle) >= 157.5:
        return 4
    if angle < -112.5 and angle > -157.5:
        return 5
    if angle < -67.5 and angle > -112.5:
        return 6
    return 7

#------------------------------------------------------------------------------
#
def b_peak_dect(data, sect, k = 5):
    y = np.array(data)

    # get its derivatives
    dy = np.array([(y[i]-y[i-1]) for i in range(1, len(y))])
    scale = 1.0/max(abs(dy.min()),abs(dy.max()))
    dy = dy*scale
    
    corner = []
    pos = 0
    valley = 0 # valley start sign

    # use the serial number of y for counting
    # start from 1 (0 has no derivative to the left)
    # end at len(u)-2 (len(u)-1 has no derivative to the right)
    for i in range(1, len(dy)-2):
        zero = 10**(-6)
        # under some threshold, take dy as 0
        if dy[i] < 0 and dy[i+1] > 0:
            # print 'acute valley'
            pos = i
            # search for the valley
            if pos < y.shape[0]-1:
                if y[pos] > y[pos+1]:
                    pos += 1
            corner.append(pos)
        # from neg to pos, or from pos to neg, means there should be 0 in between
        elif dy[i] < -zero and abs(dy[i+1]) < zero:
            # print 'down, valley start'
            valley = 1
            if pos < len(dy)-1:
                if y[pos] > y[pos+1]:
                    pos += 1
            corner.append(pos)
        elif abs(dy[i]) < zero and dy[i+1] > zero and valley == 1:
            # print 'up, valley end'
            if pos < len(dy)-1:
                if y[pos] > y[pos+1]:
                    pos += 1
            corner.append(pos)
            valley = 0

    # remove duplicated element (if there is any)
    corner = sorted(set(corner),key=corner.index)

    # #filter no. 1:
    # #threshold for speed
    # #speed should be smaller than the threshold
    # ptr = 0
    # thres_v = 0.11
    # while ptr < len(corner):
    #     temp = corner[ptr]
    #     if y[temp] > thres_v:
    #         del corner[ptr]
    #     else:
    #         ptr += 1

    # filter no. 4
    # remove extra marker in the same labanotation section
    for i in range(len(sect)):
        start = sect[i][0]
        end = sect[i][1]
        within = []
        for j in range(start, end+1):
            if j in corner:
                within.append(j)
        for j in within[1:-1]:
            corner.remove(j)

    return corner

#------------------------------------------------------------------------------
#
def b_peak_dect_no_remove(data, k = 5):
    y = np.array(data)

    # get its derivatives
    dy = np.array([(y[i]-y[i-1]) for i in range(1,len(y))])
    scale = 1.0/max(abs(dy.min()),abs(dy.max()))
    dy = dy*scale
    
    corner = []
    pos = 0
    valley = 0 # valley start sign

    # use the serial number of y for counting
    # start from 1 (0 has no derivative to the left)
    # end at len(u)-2 (len(u)-1 has no derivative to the right)
    for i in range(1,len(dy)-2):
        zero = 10**(-6)
        # under some threshold, take dy as 0
        if dy[i] < 0 and dy[i+1] > 0:
            # print 'acute valley'
            pos = i
            # search for the valley
            if pos < y.shape[0]-1:
                if y[pos] > y[pos+1]:
                    pos += 1
            corner.append(pos)
        # from neg to pos, or from pos to neg, means there should be 0 in between
        elif dy[i] < -zero and abs(dy[i+1]) < zero:
            # print 'down, valley start'
            valley = 1
            if pos < len(dy)-1:
                if y[pos] > y[pos+1]:
                    pos += 1
            corner.append(pos)
        elif abs(dy[i]) < zero and dy[i+1] > zero and valley == 1:
            # print 'up, valley end'
            if pos < len(dy)-1:
                if y[pos] > y[pos+1]:
                    pos += 1
            corner.append(pos)
            valley = 0

    #remove duplicated element (if there is any)
    corner = sorted(set(corner),key=corner.index)

    return corner

#------------------------------------------------------------------------------
#
def peak_dect(data, x_thres = 0, y_thres = 360/16.0):
    # [[peak/valley index, status],...]
    # status==-1: valley,
    # status== 1, peak.
    peaks = [[0,0]]

    grad = np.zeros(len(data)-1)
    for i in range(len(data)-1):
        grad[i] = data[i+1]-data[i]

    # (>0,<0),(>0,=0) are peaks,
    # (<0,>0),(<0,=0) are valleys,
    # (=0,=0),(=0,<0),(=0,>0) are not needed.
    for i in range(1,len(grad)-1):
        if grad[i] < 0 and grad[i+1] >= 0:
            peaks.append([i+1,-1])
        elif grad[i] > 0  and grad[i+1] <= 0:
            peaks.append([i+1,1])
        else:
            continue
    
    remove = []

    # remove peaks/valleys too close to each other based on position and value
    for i in range(len(peaks)-1,0,-1):
        t = peaks[i-1][1]   # type, peak or valley
        now = data[peaks[i][0]]
        prev = data[peaks[i-1][0]]
        if t==-1:   # valley
            # set up upper and lower threshold differently to prevent overshot/ringing
            up = y_thres * 2/3
            low = y_thres * 1/3
        else:   # peak
            up = y_thres * 1/3
            low = y_thres * 2/3
        if now < prev+up and now > prev-low:
            remove.append(peaks[i][0])
            del peaks[i]
    
#    #remove peaks/valleys based on status
#    i = 1
#    while i < len(peaks):
#        status = check_status(data[peaks[i][0]])
#        prev_status = check_status(data[peaks[i-1][0]])
#        if status == prev_status:
#            remove.append(peaks[i][0])
#            del peaks[i]
#        else:
#            i+=1

    # or you can return peaks directly
    out_peaks = [peaks[j][0] for j in range(len(peaks)) if peaks[j][1]==1]
    out_valleys = [peaks[j][0] for j in range(len(peaks)) if peaks[j][1]==-1]

    return (out_peaks, out_valleys, remove)
#    return (out_peaks, out_valleys)

#------------------------------------------------------------------------------
# set a range for all peaks/valleys.
# peak is the index. The range is either defined by angle, or time.
#
def find_range(peak, data, angle, frame = 30):
    # find the one in front
    i = peak
    while i > 0:
        if abs(data[i]-data[peak]) < angle and peak-i < frame:
            i-=1
        else:
            break
    # find the one in back
    j = peak
    while j < len(data)-1:
        if abs(data[j]-data[peak]) < angle and j-peak < frame:
            j+=1
        else:
            break

    return (i, j)

#------------------------------------------------------------------------------
#
def peak_range(data, frame = 30, angle = 360/64.0):
    # data structure: [[index, start, end],...]
    (peaks, valleys, remove) = peak_dect(data)
    index = sorted(peaks+valleys)
    out = []
    for i in range(len(index)):
        (start, end) = find_range(index[i], data, angle, frame)
        out.append([index[i],start,end])

    return out

#------------------------------------------------------------------------------
# estimate how many clusters
#
def esti_cnt(frame_cnt, data, dist):
    tmp = np.zeros(frame_cnt/(dist*2)+1)
    for i in range(len(data)):
        a = int(data[i])/(dist*2)
        tmp[a] += 1
    cnt = 0
    for i in range(len(tmp)):
        if tmp[i] > 0.5:
           cnt+=1

    return cnt

#------------------------------------------------------------------------------
# the distance from a peak to its center
#
def max_dist(data, km):
    label = km.labels_
    center = km.cluster_centers_
    tmp = []
    for i in range(len(data)):
        tmp = abs(data[i]-center[label[i]])

    return max(tmp)

#------------------------------------------------------------------------------
#
def kmeans(frame_cnt, data, dist):
    cnt = esti_cnt(frame_cnt, data, dist)
    maxi = 0
    while maxi < dist:
        print ('count of clusters: '+ str(cnt))
        km = KMeans(n_clusters=cnt).fit(data.reshape(-1,1))
        maxi = max_dist(data,km)
        
        center = km.cluster_centers_
        center.sort(axis=0)
        center=center.reshape(-1)
        diff = [center[i]-center[i-1] for i in range(1,len(center))]
        print ('minimum distance between two centers: ' + str(min(diff)))
        
        print ('maximum distance between peaks and its\' center: ' + str(maxi))
        cnt -= 1
    km = KMeans(n_clusters=cnt+2).fit(data.reshape(-1,1))
    print  ('\nmax distance:' + str(max_dist(data,km)))

    return km.cluster_centers_

#------------------------------------------------------------------------------
#
def draw(a, x, y):
    marker_p,marker_v, remove= peak_dect(a)
    pl.plot(a)
    pl.plot(a, '^', markersize=10, markevery=marker_p)
    pl.plot(a, 'v', markersize=10, markevery=marker_v)
    pl.plot(a, 'D', markersize=5, markevery=remove)

#------------------------------------------------------------------------------
#
if __name__ == '__main__' :
    pl.clf()
    f = open('dump.csv')
    line = f.readline()
    line.strip()
    elLt = []
    elLp = []
    elRt = []
    elRp = []
    wrLt = []
    wrLp = []
    wrRt = []
    wrRp = []
    while line!='':
        temp_str = line.split(',')
        elLt.append(float(temp_str[0]))
        elLp.append(float(temp_str[1]))
        elRt.append(float(temp_str[2]))
        elRp.append(float(temp_str[3]))
        wrLt.append(float(temp_str[4]))
        wrLp.append(float(temp_str[5]))
        wrRt.append(float(temp_str[6]))
        wrRp.append(float(temp_str[7]))
        line = f.readline()
        line.strip()

#    gauss = wf.gaussFilter(71,30)
#    elLt2 = wf.calcFilter(np.array(elLt), gauss)
    x = 0
    y = 360/32.0
    draw(elLt, x, y)
#    draw(elLt2, x, y)
    draw(elLp, x, y)
    draw(elRt, x, y)
    draw(elRp, x, y)
    draw(wrLt, x, y)
    draw(wrLp, x, y)
    draw(wrRt, x, y)
    draw(wrRp, x, y)
    
    mark =peak_range(elLp)
    mark+=peak_range(elLt)
    mark+=peak_range(elRp)
    mark+=peak_range(elRt)
    mark+=peak_range(wrLp)
    mark+=peak_range(wrLt)
    mark+=peak_range(wrRp)
    mark+=peak_range(wrRt)

    mark.sort()
    test = np.zeros((len(mark)))
    for i in range(len(mark)):
        test[i] = mark[i][0]

    cnt = len(elLp)/24
    while cnt > 1:
        km = KMeans(n_clusters=cnt).fit(test.reshape(-1,1))
        a = km.cluster_centers_
        a.sort(axis=0)
        b = [a[i+1]-a[i] for i in range(len(a)-1)]
        print (min(b))
        if min(b) < 24:
            cnt -= 1
        else:
            print (cnt)
            break
    for i in range(len(a)):
        pl.plot([a[i],a[i]],[-180,180],'--', linewidth = 3)

    pl.tight_layout()

    pl.plot([0,1000],[22.5,22.5],'--',color='black')
    pl.plot([0,1000],[67.5,67.5],'--',color='black')
    pl.plot([0,1000],[112.5,112.5],'--',color='black')
    pl.plot([0,1000],[157.5,157.5],'--',color='black')
    pl.plot([0,1000],[-22.5,-22.5],'--',color='black')
    pl.plot([0,1000],[-67.5,-67.5],'--',color='black')
    pl.plot([0,1000],[-112.5,-112.5],'--',color='black')
    pl.plot([0,1000],[-157.5,-157.5],'--',color='black')
