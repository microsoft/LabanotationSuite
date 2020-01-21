# --------------------------------------------------------------------------------------------
# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.
# --------------------------------------------------------------------------------------------

import sys, os, math, copy
import json

import numpy as np
from collections import OrderedDict
import labanProcessor as lp

import matplotlib.patches as patches
from matplotlib.patches import Patch

import tkMessageBox

import settings

class Algorithm:
    algorithm = None
    ax = None

    jointFrames = []
    timeS = None
    all_laban = []
    fButtonDown = False
    selectedFrame = 0
    selectedFrameMarker = None
    duration = 0.0
    data_fps = 30.0

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
        self.fButtonDown = False
        self.selectedFrame = 0
        self.selectedFrameMarker = None
        self.duration = 0.0
        self.data_fps = 30.0

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

        all_laban = []

        # get hand position
        timeS = np.zeros(cnt)

        for i in range(0, cnt):
            timeS[i] = jointD[i]['timeS'][0]

        elR = np.zeros((cnt, 3))
        elL = np.zeros((cnt, 3))
        wrR = np.zeros((cnt, 3))
        wrL = np.zeros((cnt, 3))
    
        for i in range(0, cnt):
            (elR[i], elL[i], wrR[i], wrL[i]) = lp.raw2sphere(jointD[i])

        # [right upper/elbow, right lower/wrist, left upper/elbow, left lower/wrist]
        # use coordinate2laban to generate labanotation for all frames
        for i in range(0, cnt):
            temp = []
            temp.append(lp.coordinate2laban(elR[i][1], elR[i][2]))
            temp.append(lp.coordinate2laban(wrR[i][1], wrR[i][2]))
            temp.append(lp.coordinate2laban(elL[i][1], elL[i][2]))
            temp.append(lp.coordinate2laban(wrL[i][1], wrL[i][2]))
            all_laban.append(temp)

        self.timeS = copy.copy(timeS)
        self.all_laban = copy.copy(all_laban)

        self.updateGraph()

        return (timeS, all_laban)

    #------------------------------------------------------------------------------
    #
    def setGraphAxis(self):
        self.ax.set_xlim((0, self.duration / 1000.0))

        max = (self.duration / 1000.0) * 1.02
        self.ax.set_xticks(np.arange(0, max, step=0.25)) # (max / 30.0)))
        self.ax.tick_params(axis='x', labelsize=10)

        self.ax.set_ylim((-0.25, 1.25))
        self.ax.tick_params(axis='y', labelsize=10)

    #------------------------------------------------------------------------------
    #
    def updateGraph(self):
        # clear canvas
        self.ax.clear()

        self.selectedFrameMarker = None
        self.setGraphAxis()

        cnt = len(self.jointFrames)
        if (cnt == 0):
            return

        # highlight individual frames
        yy = self.ax.get_ylim()
        padding = 0.0

        for i in range(1, cnt):
            start = self.timeS[i-1] / 1000.0
            end = self.timeS[i] / 1000.0
            isFilled = self.jointFrames[i-1]['filled'][0]

            c = 'b'
            if isFilled:
                c = 'r'

            x_width = (end - start) - padding
            if (x_width > 0):
                p = patches.Rectangle((start, yy[0]), x_width, (yy[1]-yy[0]), alpha=0.4, color=c)
                self.ax.add_patch(p)

        legend_elements = [Patch(facecolor='b', edgecolor='b', alpha=0.5, label='Original Frame'),
                           Patch(facecolor='r', edgecolor='r', alpha=0.5, label='Interpolated Frame'),
                          ]

        self.ax.legend(handles=legend_elements, bbox_to_anchor=(0, 1), loc=3, ncol=2) # , mode='expand', borderaxespad=0)

        self.setSelectedFrameMarker()

    #------------------------------------------------------------------------------
    #
    def setSelectedFrameMarker(self):
        cnt = len(self.jointFrames)
        idx = self.selectedFrame
        if ((idx is None) or (idx < 0) or (idx >= cnt)):
            return

        time = self.jointFrames[idx]['timeS'][0] / 1000.0
        padding = ((self.duration / 1000.0) / cnt) / 6.0

        if (self.selectedFrameMarker is None):
            yy = self.ax.get_ylim()
            self.selectedFrameMarker = patches.Rectangle((time-padding, yy[0]), 2*padding, (yy[1]-yy[0]), alpha=0.5, color='black')
            self.ax.add_patch(self.selectedFrameMarker)
        else:
            self.selectedFrameMarker.set_x(time-padding)

    #------------------------------------------------------------------------------
    #
    def findNearestFrameForTime(self, time):
        cnt = len(self.jointFrames)
        if (cnt == 0):
            return None

        timeMS = time * 1000.0

        # find the frame corresponding to the given time
        for idx in range(0, cnt):
            kt = self.timeS[idx]

            if (kt == timeMS):
                return idx
            elif (kt > timeMS):
                break

        # should not get here if idx == 0, but let's check anyway
        if (idx == 0):
            return idx

        # now that we have an index, determine which frame time is closest to
        dist1 = abs(kt - time)
        dist2 = abs(self.timeS[idx-1] - time)

        return idx if (dist1 < dist2) else (idx-1)

    #------------------------------------------------------------------------------
    #
    def saveToJSON(self):
        filePath = settings.checkFileAlreadyExists(settings.application.outputFilePathJson, fileExt=".json", fileTypes=[('json files', '.json'), ('all files', '.*')])
        if (filePath is None):
            return

        # save json script
        file_name = os.path.splitext(os.path.basename(filePath))[0]

        labandata = OrderedDict()
        positions = []

        # generate labanotation json data structure
        for i in range(len(self.timeS)):
            if i==(len(self.timeS)-1):
                dur = '-1'
            else:
                dur = '1'

            time = self.timeS[i]

            positions.append("Position"+str(i))
            data = OrderedDict()
            data["start time"] = [str(time)]
            data["duration"] = [dur]
            data["head"] = ['Forward','Normal']
            data["right elbow"] = [self.all_laban[i][0][0], self.all_laban[i][0][1]]
            data["right wrist"] = [self.all_laban[i][1][0], self.all_laban[i][1][1]]
            data["left elbow"] = [self.all_laban[i][2][0], self.all_laban[i][2][1]]
            data["left wrist"] = [self.all_laban[i][3][0], self.all_laban[i][3][1]]
            data["rotation"] = ['ToLeft','0']
            labandata[positions[i]] = data

        labanjson = OrderedDict()
        labanjson[file_name] = labandata

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
        time = time * (self.duration / 1000.0)
        self.selectedFrame = self.findNearestFrameForTime(time)
        self.setSelectedFrameMarker()

    # -----------------------------------------------------------------------------
    # canvas click event
    #
    def onCanvasClick(self, event):
        if (event.xdata is None) or (event.ydata is None):
            return

        # map xdata to [0..1]
        xx = self.ax.get_xlim()
        p = event.xdata / (xx[1]-xx[0])

        # call application so that other graphs can be updated as well
        settings.application.selectTime(p)

        self.fButtonDown = True

    # -----------------------------------------------------------------------------
    # canvas click release event
    #
    def onCanvasRelease(self, event):
        self.fButtonDown = False

    # -----------------------------------------------------------------------------
    # canvas move event
    #
    def onCanvasMove(self, event):
        if (self.fButtonDown):
            # map xdata to [0..1]
            xx = self.ax.get_xlim()
            p = event.xdata / (xx[1]-xx[0])

            # call application so that other graphs can be updated as well
            settings.application.selectTime(p)

