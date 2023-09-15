# --------------------------------------------------------------------------------------------
# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.
# --------------------------------------------------------------------------------------------

import sys, os, math, copy
import json
from math import sin, cos, sqrt, radians

import numpy as np
from decimal import Decimal
from collections import OrderedDict

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Patch
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

import settings

from . import labanProcessor as lp

import kp_extractor as kpex

import accessory as ac
import wavfilter as wf
import cluster as cl

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
    default_gauss_sigma = 5

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
        self.vlines = None

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
    def convertToLabanotation(self, ax, jointD, forceReset):
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

            self.line_ene = None
            self.vlines = None
            self.y_data = []
            self.points = []

        self.calculateUnfilteredLaban()

        return self.totalEnergy()

    #------------------------------------------------------------------------------
    # unfiltered labanotation
    #
    def calculateUnfilteredLaban(self):
        cnt = len(self.jointFrames)

        # get hand position
        self.unfilteredTimeS = np.zeros(cnt)

        elR = np.zeros((cnt, 3))
        elL = np.zeros((cnt, 3))
        wrR = np.zeros((cnt, 3))
        wrL = np.zeros((cnt, 3))
        
        for i in range(0, cnt):
            self.unfilteredTimeS[i] = self.jointFrames[i]['timeS'][0]

            (elR[i], elL[i], wrR[i], wrL[i]) = lp.raw2sphere(self.jointFrames[i])

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

    #------------------------------------------------------------------------------
    # apply total energy algoritm to joint data frames and calculate labanotation
    #
    def totalEnergy(self):
        cnt = len(self.jointFrames)

        handR = np.zeros((cnt, 3))
        handL = np.zeros((cnt, 3))

        for i in range(0, cnt):
            handR[i][0] = self.jointFrames[i]['wristR']['x'][0] # meters to centimeters
            handR[i][1] = self.jointFrames[i]['wristR']['y'][0]
            handR[i][2] = self.jointFrames[i]['wristR']['z'][0]
            handL[i][0] = self.jointFrames[i]['wristL']['x'][0]
            handL[i][1] = self.jointFrames[i]['wristL']['y'][0]
            handL[i][2] = self.jointFrames[i]['wristL']['z'][0]

        # filtered by a Gaussian filter with window-size of 101 and sigma of 10
        # window-size of 61 also works
        gauss_window_size = self.gauss_window_size
        gauss_large_sigma = self.gauss_sigma
        gauss_small_sigma = 1

        gauss = wf.gaussFilter(gauss_window_size, gauss_large_sigma)
        handRF = wf.calcFilter(handR, gauss)
        handLF = wf.calcFilter(handL, gauss)

        handRv = ac.vel(self.unfilteredTimeS, handRF)
        handLv = ac.vel(self.unfilteredTimeS, handLF)
        handRa = ac.acc(self.unfilteredTimeS, handRv)
        handLa = ac.acc(self.unfilteredTimeS, handLv)
        
        # calculate energy
        energy = kpex.energy_function_ijcv(v_l=handLv, a_l=handLa, v_r=handRv, a_r=handRa)

        # calculate energy again with gauss_small_sigma
        gauss_small = wf.gaussFilter(gauss_window_size, gauss_small_sigma)
        handRF_small = wf.calcFilter(handR, gauss_small)
        handLF_small = wf.calcFilter(handL, gauss_small)

        handRv_small = ac.vel(self.unfilteredTimeS, handRF_small)
        handLv_small = ac.vel(self.unfilteredTimeS, handLF_small)
        handRa_small = ac.acc(self.unfilteredTimeS, handRv_small)
        handLa_small = ac.acc(self.unfilteredTimeS, handLv_small)
        
        # calculate energy
        energy_small = kpex.energy_function_ijcv(v_l=handLv_small, a_l=handLa_small, v_r=handRv_small, a_r=handRa_small)

        indices = kpex.gaussian_pecdec(energy)

        self.y_data = []
        self.y_data = energy

        self.points = {}
        self.points = dict(zip(indices, self.y_data[indices]))

        if (self.ax != None):
            xmax = max(self.unfilteredTimeS) / 1000.0

            self.ax.plot(energy, color='dimgray', label='Total')
            self.ax.plot(energy_small, color='mediumpurple', label='Naive')
       
            self.ax.set_xlim((0, len(energy)-1))
            self.ax.set_ylim((min(energy)-0.5, max(energy)+0.5))

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

            self.ax.legend(handles=legend_elements, bbox_to_anchor=(0, 1), loc=3, ncol=7) # , mode='expand', borderaxespad=0)

        self.updateEnergyPlotAndLabanScore(True)
        self.highlightLabanotationRegions(self.unfilteredLaban, (min(energy)-0.5, max(energy)+0.5))

        # additional energy markers
        if (self.ax != None):
            corner,_,_ = cl.peak_dect(energy, y_thres=0)
            self.ax.plot(energy, '.', color = 'slategrey', mew=3, markersize=14, markevery=corner) # bottom

            infl = ac.inflection(energy)
            self.ax.plot(energy, '.', color = 'k', mew=3, markersize=12, markevery=infl)

        self.setSelectedFrameMarker()

        return (self.timeS, self.all_laban)

    #------------------------------------------------------------------------------
    # plot different colors for each labanotation region.
    #
    def highlightLabanotationRegions(self, laban, y):
        if (self.ax == None):
            return

        laban_sect = ac.split(laban)
        cnt = len(laban)

        for i in range(len(laban_sect)):
            start = laban_sect[i][0]
            end = laban_sect[i][1]

            # color and alpha
            c = 'wheat'
            a = 0.4
            if start == end:
                c = 'tan'
                a = 0.4

            # (x_start, y_start), x_width, y_width, alpha
            if (i < len(laban_sect) - 1):
                x_width = end - start + 0.5
            else:
                x_width = cnt - start + 0.25

            p = patches.Rectangle((start-0.25, y[0]), x_width, y[1]-y[0], alpha=a, color=c)
            self.ax.add_patch(p)

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
    # update energy markers and lines, and labanotation score
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

            # plot vertical lines to denote labanotation keyframes
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
        if (self.ax is None):
            return

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

