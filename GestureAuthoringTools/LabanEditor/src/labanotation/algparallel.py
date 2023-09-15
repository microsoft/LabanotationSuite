# --------------------------------------------------------------------------------------------
# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.
# --------------------------------------------------------------------------------------------

import sys, os, math, copy
import json
from math import sin, cos, sqrt, radians
from operator import sub

import numpy as np
from decimal import Decimal
from collections import OrderedDict

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Patch
from matplotlib.patches import BoxStyle
from matplotlib.widgets import Slider, Cursor, Button
from matplotlib.backend_bases import MouseEvent
import matplotlib.patches as patches
import matplotlib.ticker as ticker

try:
    from tkinter import messagebox
except ImportError:
    # Python 2
    import tkMessageBox as messagebox

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tool'))
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'laban_tool'))

import settings

from . import labanProcessor as lp

from tool import accessory as ac
from tool import wavfilter as wf
from tool import cluster as cl


class Algorithm:
    algorithm = None
    ax = None

    jointFrames = []

    timeS = None
    all_laban = []

    unfilteredTimeS = None
    unfilteredLaban = []

    labandata = OrderedDict()

    line_ene = None
    vlines = None
    y_data = []
    points = []

    data_fps = 30
    dragging_sb = False
    dragging_point = None

    selectedFrame = 0
    selectedFrameMarker = None

    default_gauss_window_size = 31
    default_gauss_sigma = 3

    gauss_window_size = default_gauss_window_size
    gauss_sigma = default_gauss_sigma

    #------------------------------------------------------------------------------
    # Class initialization
    #
    def __init__(self, algorithm):
        self.algorithm = algorithm

    #------------------------------------------------------------------------------
    # reset class variables
    #
    def reset(self):
        self.jointFrames = []

        self.timeS = None
        self.all_laban = []

        self.unfilteredTimeS = None
        self.unfilteredLaban = []

        self.labandata = OrderedDict()

        self.line_ene = None
        self.y_data = []
        self.points = []

        self.data_fps = 30
        self.dragging_sb = False
        self.dragging_point = None

        self.selectedFrame = 0
        self.selectedFrameMarker = None

    #------------------------------------------------------------------------------
    # convert joint data frames to labanotation
    #
    def convertToLabanotation(self, ax, jointD, forceReset, base_rotation_style='every'):
        if (forceReset):
            self.reset()

        self.ax = ax

        self.jointFrames = copy.copy(jointD)

        cnt = len(jointD)

        self.data_fps = 30
        self.duration = jointD[cnt-1]['timeS'][0] if (cnt > 0) else 0.0

        # clear canvas
        if (self.ax != None):
            self.ax.clear()
            self.selectedFrameMarker = None
            self.ax.get_yaxis().set_visible(False)

            # determine text height
            figW, figH = self.ax.get_figure().get_size_inches()
            _, _, w, h = self.ax.get_position().bounds
            disp_ratio = (figH * h) / (figW * w)
            data_ratio = sub(*self.ax.get_ylim()) / sub(*self.ax.get_xlim())

            tmp, self.textHeight = settings.getTextExtent(self.ax, 'Tyg')
            self.textHeight = self.textHeight * self.ax.get_figure().dpi * disp_ratio

        self.calculateUnfilteredLaban(base_rotation_style=base_rotation_style)

        return self.parallelEnergy()

    #------------------------------------------------------------------------------
    # unfiltered labanotation
    #
    def calculateUnfilteredLaban(self, base_rotation_style='every'):
        base_rotation = None
        if base_rotation_style == 'first':
            base_rotation = lp.calculate_base_rotation(
                self.jointFrames[0])

        cnt = len(self.jointFrames)

        # get hand position
        self.unfilteredTimeS = np.zeros(cnt)

        elR = np.zeros((cnt, 3))
        elL = np.zeros((cnt, 3))
        wrR = np.zeros((cnt, 3))
        wrL = np.zeros((cnt, 3))
    
        for i in range(0, cnt):
            if base_rotation_style == 'every':
                base_rotation = lp.calculate_base_rotation(self.jointFrames[i])
            self.unfilteredTimeS[i] = self.jointFrames[i]['timeS'][0]

            (elR[i], elL[i], wrR[i], wrL[i]) = lp.raw2sphere(
                self.jointFrames[i],
                base_rotation=base_rotation)

        # [right upper/elbow, right lower/wrist, left upper/elbow, left lower/wrist]
        # use coordinate2laban to generate labanotation for all frames
        self.unfilteredLaban = []

        for i in range(0, cnt):
            temp = []
            temp.append(lp.coordinate2laban(elR[i][1], elR[i][2]))
            temp.append(lp.coordinate2laban(wrR[i][1], wrR[i][2]))
            temp.append(lp.coordinate2laban(elL[i][1], elL[i][2]))
            temp.append(lp.coordinate2laban(wrL[i][1], wrL[i][2]))
            self.unfilteredLaban.append(temp)

        self.lines = \
            [[elR, [self.unfilteredLaban[i][0] for i in range(cnt)], 'b', "Right Elbow"],\
             [wrR, [self.unfilteredLaban[i][1] for i in range(cnt)], 'c', "Right Wrist"],\
             [elL, [self.unfilteredLaban[i][2] for i in range(cnt)], 'y', "Left Elbow"],\
             [wrL, [self.unfilteredLaban[i][3] for i in range(cnt)], 'm', "Left Wrist"]]

    #------------------------------------------------------------------------------
    # apply parallel energy algoritm to joint data frames and calculate labanotation
    #
    def parallelEnergy(self):
        cnt = len(self.jointFrames)

        gauss_window_size = self.gauss_window_size
        gauss_large_sigma = self.gauss_sigma
        gauss_small_sigma = 1

        valley_output = []
        offset = 0

        offset, energy = self.plot_parallel_energy(valley_output, self.lines, offset, gauss_window_size, gauss_large_sigma, gauss_small_sigma, cnt)

        # count how many peaks are there in a frame
        cnter = np.zeros(cnt)
        for i in range(4):
            for j in valley_output[i]:
                cnter[j] += 1

        # merge keyframes here
        # two thresholds are used
        # 1. distance between two potential keyframes
        # 9 x 33 m sec
        # 
        # allegro (max 168M) - 350 m sec
        # prestissimo - 200BPM - 300 m sec
        # |--- 150 ---|--- 150 ---|
        #       |--- 300 ---|

        # for sample, fps = 30
        # 33m sec x 9 = 300

        # for human3.6m, fps = 50
        # 20m sec x 15 = 300

        thres_dis = 9 # int(300/(1000/self.data_fps))

        # use a sliding window searching among sparse valleys
        indices = []
        ptr = 0
        while ptr < cnt:
            tmp = cnter[ptr]
            if tmp > 0:
                indices.append(ptr)
                ptr += thres_dis
            else:
                ptr += 1

        new_indices = []
        valley_output_new = []
        for i in valley_output:
            tmp = []
            for j in i:
                tmp.append(j)
            valley_output_new.append(tmp)
        for i in range(len(indices)):
            left = indices[i]
            right = indices[i]+thres_dis
            group = []
            for j in range(4):
                ptr = 0
                while ptr < len(valley_output_new[j]):
                    tmp = valley_output_new[j][ptr]
                    if tmp <= right and tmp >= left:
                        group.append(tmp)
                        del valley_output_new[j][ptr]
                    ptr += 1
            if len(group) == 0:
                continue
            # kf = sum(group)/len(group)
            #the first key frame, use the first potential frame in its group
            if i == 0:
                group.sort()
                kf = group[0]
            #the last key frame, use the last potential frame in its group
            elif i == len(indices)-1:
                group.sort()
                kf = group[-1]
            else:
                kf = sum(group)/len(group)
            new_indices.append(kf)

        new_indices = list(set(new_indices))
        new_indices.sort()

        self.y_data = energy
        self.points = dict(zip(new_indices, self.y_data[new_indices]))

        if (self.ax != None):
            xmax = max(self.unfilteredTimeS) / 1000.0

            self.ax.set_xlim((0, cnt-1))
            self.ax.set_ylim((0, offset + 0.5))

            def format_func(value, tick_number):
                cnt = len(self.unfilteredTimeS)
                idx = int(value)
                if (idx < 0) or (idx >= cnt):
                    return ""

                time = self.unfilteredTimeS[idx] / 1000.0

                return r"${:.2f}$".format(time)

            # look at https://matplotlib.org/3.1.1/gallery/ticks_and_spines/tick-locators.html for fine-tuning ticks
            self.ax.xaxis.set_major_formatter(plt.FuncFormatter(format_func))

            self.ax.tick_params(axis='y', labelsize=8)

            legend_elements = [Line2D([0], [0], color='dimgray', label='Energy'),
                               Line2D([0], [0], color='mediumpurple', label='Naive Energy'),
                               Patch(facecolor='wheat', edgecolor='wheat', alpha=0.4, label='Labanotation Frame Blocks'),
                               Patch(facecolor='tan', edgecolor='tan', alpha=0.4, label='Labanotation Transition Block'),
                               Line2D([0], [0], marker='o', color='w', label='Peaks', markerfacecolor='slategrey', markersize=10),
                               Line2D([0], [0], marker='o', color='w', label='Inflection', markerfacecolor='k', markersize=10),
                               Line2D([0], [0], marker='*', color='w', label='Labanotation Key Frames', markerfacecolor='g', markersize=16)]

            self.ax.legend(handles=legend_elements, bbox_to_anchor=(0, 1), loc=3, ncol=7) #, mode='expand', borderaxespad=0)

            self.line_ene, = self.ax.plot(self.y_data, '*', color='g', mew=3, markersize=14, markevery=new_indices)

            xs = np.array((new_indices, ) if np.isscalar(new_indices) else new_indices, copy=False)
            lims = self.ax.get_ylim()
            x_points = np.repeat(xs[:, None], repeats=3, axis=1).flatten()
            y_points = np.repeat(np.array(lims + (np.nan, ))[None, :], repeats=len(xs), axis=0).flatten()
            self.vlines, = self.ax.plot(x_points, y_points, scaley = False, color='g')

        # force a redraw
        if (self.ax != None):
            self.ax.figure.canvas.draw()

        self.updateLaban(new_indices)

        self.setSelectedFrameMarker()

        return (self.timeS, self.all_laban)

    #------------------------------------------------------------------------------
    #   t: theta, p: phi
    #   dt:theta', dp: phi'
    #   v = sqrt[(p' * sint)^2+(t')^2]
    #
    def calc_geodesic_ang_speed(self, angles):
        cnt = len(angles)
        v = np.zeros(cnt)

        t = np.array([angles[i][1] for i in range(cnt)]) # theta
        p = np.array([angles[i][2] for i in range(cnt)]) # phi
        # generate parameters
        dt = [t[i+1]-t[i] for i in range(0,cnt-1)] # theta
        dt.append(dt[-1])
        dp = [p[i+1]-p[i] for i in range(0,cnt-1)] # phi
        dp.append(dp[-1])

        for i in range(cnt):
            v_1 = dp[i] * sin(t[i])
            v_2 = dt[i]
            v[i] = sqrt( v_1**2 + v_2**2 ) # v = sqrt[(p' * sint)^2+(t')^2

        return v

    #------------------------------------------------------------------------------
    #
    def getLabanotationKeyframeData(self, idx, time, dur, laban):
        data = OrderedDict()
        data["start time"] = [str(time)]
        data["duration"] = [str(dur)]
        data["head"] = ['Forward','Normal']
        data["right elbow"] = [laban[0][0], laban[0][1]]
        data["right wrist"] = [laban[1][0], laban[1][1]]
        data["left elbow"] = [laban[2][0], laban[2][1]]
        data["left wrist"] = [laban[3][0], laban[3][1]]
        data["rotation"] = ['ToLeft','0']

        return data

    #------------------------------------------------------------------------------
    # update labanotation key frames
    #
    def updateLaban(self, indices):
        self.labandata = OrderedDict()
        positions = []

        self.timeS = []
        self.all_laban = []

        # generate labanotation json data structure
        idx = 0
        cnt = len(indices)

        if (cnt == 0):
            return

        for i in range(cnt):
            j = indices[i]

            # add an initial labanotation keyframe
            if ((i==0) and (j != i)):
                time = int(self.unfilteredTimeS[i])
                dur = 1

                # store new time and laban
                self.timeS.append(time)
                self.all_laban.append(self.unfilteredLaban[i])

                positions.append("Position"+str(i))
                self.labandata[positions[idx]] = self.getLabanotationKeyframeData(idx, time, dur, self.unfilteredLaban[i])
                idx = idx + 1

            time = int(self.unfilteredTimeS[j])

            if (j == (cnt-1)):
                dur = '-1'
            else:
                dur = '1'

            # store new time and laban
            self.timeS.append(time)
            self.all_laban.append(self.unfilteredLaban[j])

            positions.append("Position"+str(i))
            self.labandata[positions[idx]] = self.getLabanotationKeyframeData(idx, time, dur, self.unfilteredLaban[j])
            idx = idx + 1

        # add a final labanotation keyframe
        i = len(self.unfilteredLaban) - 1
        j = indices[cnt - 1]
        if (j != i):
            time = int(self.unfilteredTimeS[i])
            dur = '-1'

            # store new time and laban
            self.timeS.append(time)
            self.all_laban.append(self.unfilteredLaban[i])

            positions.append("Position"+str(i))
            self.labandata[positions[idx]] = self.getLabanotationKeyframeData(idx, time, dur, self.unfilteredLaban[i])
            idx = idx + 1

    #------------------------------------------------------------------------------
    #
    def getTextSize(self, renderer, text, properties, transData):
        (w, h, d) = renderer.get_text_width_height_descent(text, properties, False)
        if (True):
            (x1, y1) = transData.inverted().transform((0, 0))
            (x2, y2) = transData.inverted().transform((w, h+d))
        else:
            (x1, y1) = transData.transform((0, 0))
            (x2, y2) = transData.transform((w, h+d))

        return (x2-x1, y2-y1)

    #------------------------------------------------------------------------------
    # apply parallel energy method
    #
    def plot_parallel_energy(self, valley_output, lines, offset, g_w_size, g_l_sigma, g_s_sigma, cnt):
        # add space for the labanotation key frames
        dy = (3 * self.textHeight)
        offset += dy

        for i in range(len(lines)):
            line = lines[i]
            sect = ac.split(line[1])

            v_tmp = self.calc_geodesic_ang_speed(line[0])

            gauss_large = wf.gaussFilter(g_w_size, g_l_sigma)
            v_large = wf.calcFilter(v_tmp, gauss_large)

            gauss_small = wf.gaussFilter(g_w_size, g_s_sigma)
            v_small = wf.calcFilter(v_tmp, gauss_small)

            v_max = v_large.max() if v_large.max() > v_small.max() else v_small.max()

            if (self.ax != None):
                self.ax.plot(v_large+offset, label = line[3], color='dimgray') # , color=line[2]
                self.ax.plot(v_small+offset, 'mediumpurple', label = line[3]) # line[2]+'-'

            # highlight labanotation frames for each body part
            for j in range(len(sect)):
                y = [offset, offset + v_max]
                start = sect[j][0]
                end = sect[j][1]

                if (self.ax != None):
                    # color and alpha
                    c = 'wheat'
                    a = 0.4
                    if start == end:
                        c = 'tan'
                        a = 0.4

                    # (x_start, y_start), x_width, y_width, alpha
                    if j < len(sect)-1:
                        x_width = end-start+0.5
                    else:
                        x_width = cnt-start+0.25

                    p = patches.Rectangle((start-0.25, y[0]), x_width, y[1]-y[0], alpha=a, color=c)

                    self.ax.add_patch(p)

            corner = cl.b_peak_dect(v_large,sect)
            if (self.ax != None):
                self.ax.plot(v_large+offset, '.', color = 'slategrey', mew=3, markersize=14, markevery=corner) # bottom

            v_d = ac.der(v_large)
            v_d = 10.0 * v_d/(max(v_d)-min(v_d))

            v_dd = ac.der(ac.der(v_large))
            v_dd = 10.0 * v_dd/(max(v_dd)-min(v_dd))

            v_ddd = ac.der(ac.der(ac.der(v_large)))
            v_ddd = 10.0 * v_ddd/(max(v_ddd)-min(v_ddd))

            infl = ac.inflection(v_large)

            if (self.ax != None):
                self.ax.plot(v_large+offset, '.', color = 'k', mew=3, markersize=12, markevery=infl)

            # search the minimum point along the narrow gaussian within the boundary
            # defined by the inflection pairs correspondng the the
            # minimum point of the wider gaussian
            real_corner = []
            for j in corner:
                right = 0
                left = 0
                for k in range(len(infl)):
                    if infl[k] > j:
                        left = infl[k-1]
                        right = infl[k]
                        break
                min_val = v_small[left]
                min_ptr = left
                for k in range(left,right+1):
                    if v_small[k] < min_val:
                        min_val = v_small[k]
                        min_ptr = k
                real_corner.append(min_ptr)

            y_data = v_small+offset
            # self.ax.plot(v_small+offset, '+', color='r', mew=3, markersize=14, markevery=real_corner)
            # for j in corner:
            #      self.ax.text(j+2, v_large[j]+offset, str("{:.1f}".format(v_large[j])) + ',\n ' + str(j))

            valley_output.append(real_corner)
            offset += v_max # to devide graphs

            t = self.ax.text(0.0, offset + (self.textHeight * 0.1), '  ' + line[3], horizontalalignment='left', verticalalignment='bottom', transform=self.ax.transData, color='black')
            offset += self.textHeight

        # reset y_data to key frame bottom frame
        for i in range(len(y_data)):
            y_data[i] = (dy / 2.0)

        p = patches.Rectangle((0.0, 0.0), cnt, dy, alpha=0.05, color='g')
        self.ax.add_patch(p)

        self.updateEnergyPlotAndLabanScore(True)

        return offset, y_data

    #------------------------------------------------------------------------------
    # update plot and labanotation score
    #
    def updateEnergyPlotAndLabanScore(self, updateLabanScore=False):
        if (self.ax != None):
            if not self.points:
                return

            x, y = zip(*sorted(self.points.items()))

            if not self.line_ene:
                # Add new plot
                self.line_ene, = self.ax.plot(self.y_data, '*', color = 'g', mew=3, markersize=14, markevery=list(x))
            else:
                # Update current plot
                self.line_ene.set_data(range(len(self.y_data)),self.y_data)
                self.line_ene.set_markevery(list(x))
                self.ax.draw_artist(self.line_ene)

            # update vertical line markers
            xs = list(x)
            xs = np.array((xs, ) if np.isscalar(xs) else xs, copy=False)
            lims = self.ax.get_ylim()
            x_points = np.repeat(xs[:, None], repeats=3, axis=1).flatten()
            y_points = np.repeat(np.array(lims + (np.nan, ))[None, :], repeats=len(xs), axis=0).flatten()
            if not self.vlines:
                # Add new plot
                self.vlines, = self.ax.plot(x_points, y_points, scaley = False, color='g')
            else:
                # Update current plot
                self.vlines.set_data(x_points, y_points)
                self.ax.draw_artist(self.vlines)

            self.ax.figure.canvas.draw_idle()

        # update laban score
        if (updateLabanScore) and (self.points):
            tmp_indices, _ = zip(*sorted(self.points.items()))
            new_indices = list(tmp_indices)

            self.updateLaban(new_indices)

            settings.application.updateLaban(self.timeS, self.all_laban)

    #------------------------------------------------------------------------------
    #
    def add_point(self, x, y=None):
        if isinstance(x, MouseEvent):
            x, y = int(x.xdata), int(x.ydata)

        y_on_curve = self.y_data[x]
        self.points[x] = y_on_curve

        return x, y_on_curve

    #------------------------------------------------------------------------------
    #
    def remove_point(self, x, _):
        if x in self.points:
            self.points.pop(x)

    #------------------------------------------------------------------------------
    #
    def setSelectedFrameMarker(self):
        cnt = len(self.jointFrames)
        idx = self.selectedFrame
        if ((idx is None) or (idx < 0) or (idx >= cnt)):
            return

        time = idx
        padding = 1.0 / 6.0

        if (self.selectedFrameMarker is None):
            yy = self.ax.get_ylim()
            self.selectedFrameMarker = patches.Rectangle((time-padding, yy[0]), 2*padding, (yy[1]-yy[0]), alpha=0.5, color='purple')
            self.ax.add_patch(self.selectedFrameMarker)
        else:
            self.selectedFrameMarker.set_x(time-padding)

    #------------------------------------------------------------------------------
    #
    def findNearestFrameForTime(self, time):
        cnt = len(self.jointFrames)
        if (cnt == 0):
            return None

        timeMS = time

        # find the frame corresponding to the given time
        for idx in range(0, cnt):
            kt = self.unfilteredTimeS[idx]

            if (kt == timeMS):
                return idx
            elif (kt > timeMS):
                break

        # should not get here if idx == 0, but let's check anyway
        if (idx == 0):
            return idx

        # now that we have an index, determine which frame time is closest to
        dist1 = abs(kt - time)
        dist2 = abs(self.unfilteredTimeS[idx-1] - time)

        return idx if (dist1 < dist2) else (idx-1)

    #------------------------------------------------------------------------------
    #
    def saveToJSON(self):
        filePath = settings.checkFileAlreadyExists(settings.application.outputFilePathJson, fileExt=".json", fileTypes=[('json files', '.json'), ('all files', '.*')])
        if (filePath is None):
            return

        # save json script
        file_name = os.path.splitext(os.path.basename(filePath))[0]

        labanjson = OrderedDict()
        labanjson[file_name] = self.labandata

        try:
            with open(filePath,'w') as file:
                json.dump(labanjson, file, indent=2)
                settings.application.logMessage("Labanotation json script was saved to '" + settings.beautifyPath(filePath) + "'")
        except Exception as e:
            strError = e
            settings.application.logMessage("Exception saving Labanotation json script to '" + settings.beautifyPath(filePath) + "': " + str(e))

    #------------------------------------------------------------------------------
    #
    def saveToTXT(self):
        filePath = settings.checkFileAlreadyExists(settings.application.outputFilePathTxt, fileExt=".txt", fileTypes=[('text files', '.txt'), ('all files', '.*')])
        if (filePath is None):
            return

        # save text script
        script = settings.application.labanotation.labanToScript(self.timeS, self.all_laban)

        try:
            with open(filePath,'w') as file:
                file.write(script)
                file.close()
                settings.application.logMessage("Labanotation text script was saved to '" + settings.beautifyPath(filePath) + "'")
        except Exception as e:
            strError = e
            settings.application.logMessage("Exception saving Labanotation text script to '" + settings.beautifyPath(filePath) + "': " + str(e))

    #------------------------------------------------------------------------------
    #
    def selectTime(self, time):
        time = time * (self.duration)
        self.selectedFrame = self.findNearestFrameForTime(time)
        self.setSelectedFrameMarker()

    #------------------------------------------------------------------------------
    # find point closest to mouse position
    #
    def find_neighbor_point(self, event):
        distance_threshold = 3.0
        nearest_point = None
        min_distance = math.sqrt(2 * (100 ** 2))
        for x, y in self.points.items():
            distance = math.hypot(event.xdata - x, event.ydata - y) # euclidian norm
            if distance < min_distance:
                min_distance = distance
                nearest_point = (x, y)
        if min_distance < distance_threshold:
            return nearest_point
        return None

    # -----------------------------------------------------------------------------
    # canvas click event
    #
    def onCanvasClick(self, event):
        if (event.xdata is None) or (event.ydata is None):
            return

        # callback method for mouse click event
        # left click
        if event.button == 1 and event.inaxes in [self.ax]:
            if event.dblclick:
                pass
            else:
                self.dragging_sb = True

                # map xdata to [0..1]
                xx = self.ax.get_xlim()
                p = (event.xdata) / (xx[1]-xx[0])

                # call application so that other graphs can be updated as well
                settings.application.selectTime(p)

        # right click
        elif event.button == 3 and event.inaxes in [self.ax]:
            point = self.find_neighbor_point(event)
            if point and event.dblclick:
                self.remove_point(*point)
            elif point:
                self.dragging_point = point
                self.remove_point(*point)
            else:
                self.add_point(event)

            self.updateEnergyPlotAndLabanScore(True)

    # -----------------------------------------------------------------------------
    # canvas click release event
    #
    def onCanvasRelease(self, event):
        if event.button == 1 and event.inaxes in [self.ax] and self.dragging_sb:
            self.dragging_sb = False
        if event.button == 3 and event.inaxes in [self.ax] and self.dragging_point:
            self.add_point(event)
            self.dragging_point = None
            self.updateEnergyPlotAndLabanScore(True)

    # -----------------------------------------------------------------------------
    # canvas move event
    #
    def onCanvasMove(self, event):
        if (not self.dragging_sb or event.xdata is None) and (not self.dragging_point):
            return

        if self.dragging_sb:
            # map xdata to [0..1]
            xx = self.ax.get_xlim()
            p = event.xdata / (xx[1]-xx[0])

            # call application so that other graphs can be updated as well
            settings.application.selectTime(p)
        else:
            self.remove_point(*self.dragging_point)
            self.dragging_point = self.add_point(event)
            self.updateEnergyPlotAndLabanScore()

    #------------------------------------------------------------------------------
    #
