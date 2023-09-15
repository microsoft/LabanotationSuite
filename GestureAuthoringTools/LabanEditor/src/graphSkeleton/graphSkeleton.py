# --------------------------------------------------------------------------------------------
# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.
# --------------------------------------------------------------------------------------------

import os, math, copy
import numpy as np

import matplotlib.pyplot as plt
plt.rcParams['toolbar'] = 'None'
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backend_tools import ToolBase, ToolToggleBase
from matplotlib.widgets import Slider, Button, RadioButtons
from mpl_toolkits.axes_grid1.inset_locator import InsetPosition
import matplotlib.patches as patches
import matplotlib.ticker as ticker

try:
    from tkinter import messagebox
except ImportError:
    # Python 2
    import tkMessageBox as messagebox

import settings

class graph3D:
    fig = None
    ax = None
    jointFrames = []    # all the frames
    all_laban = []
    timeS = []

    joints = []         # temporary joints to be drawn

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
                    'formats':[ int,
                                bool,
                                jType, jType,
                                jType, jType,
                                jType, jType, jType, jType,
                                jType, jType, jType, jType,
                                jType, jType, jType, jType,
                                jType, jType, jType, jType,
                                jType, jType, jType, jType, jType]})

    isInterpolatedKeyFrame = False
    annSelection = None

    #------------------------------------------------------------------------------
    # Class initialization
    #
    def __init__(self):
        self.strTitle = '3D Joint Data'
        self.fig = plt.figure()
        self.fig.canvas.set_window_title(self.strTitle)
        self.fig.set_size_inches((settings.screen_cx * 0.49) / self.fig.dpi, (settings.screen_cy * 0.465) / self.fig.dpi)

        self.fig.canvas.mpl_connect('resize_event', self.onresize)
        self.fig.canvas.mpl_connect('close_event', self.onclose)

        offset=0.08
        self.ax = Axes3D(self.fig, rect=[0, offset, 1.0, 1.0-offset]) # (left, bottom, width, height)
        self.ax.view_init(10, 10)
        self.ax.dist = 9

        self.axSliderTime = plt.axes([0.08, 0.055, 0.80, 0.04], facecolor='lightgoldenrodyellow')
        self.slidertime = Slider(self.axSliderTime, 'Time', 0.0, 1.0, valinit=0, valstep=0.005, valfmt='%0.03fs')
        self.cidSlider = self.slidertime.on_changed(self.onsliderupdate)

        # set currect axes context back to main axes self.ax
        plt.sca(self.ax)

        offset = 0.112
        rect = [0.08, offset + 0.036, 0.80, 0.03]
        self.axFrameBlocks1 = plt.axes(rect)

        rect = [0.08, offset, 0.80, 0.03]
        self.axFrameBlocks2 = plt.axes(rect)

        # set currect axes context back to main axes self.ax
        plt.sca(self.ax)
        
        # add empty joints placeholders
        # [x, y, z, father joint] serial number is the same as Kinect
        self.joints.append([0,0,0,-1])  # spineBase,#0
        self.joints.append([0,0,0,0])   # spineMid,#1
        self.joints.append([0,0,0,20])  # neck,#2
        self.joints.append([0,0,0,2])   # head, #3
        self.joints.append([0,0,0,20])  # shoulderLeft, #4
        self.joints.append([0,0,0,4])   # elbowLeft, #5
        self.joints.append([0,0,0,5])   # wristLeft, #6
        self.joints.append([0,0,0,6])   # handLeft, #7
        self.joints.append([0,0,0,20])  # shoulderRight, #8
        self.joints.append([0,0,0,8])   # elbowRight, #9
        self.joints.append([0,0,0,9])   # wristRight, #10
        self.joints.append([0,0,0,10])  # handRight, #11
        self.joints.append([0,0,0,0])   # hipLeft, #12
        self.joints.append([0,0,0,12])  # kneeLeft, #13
        self.joints.append([0,0,0,13])  # ankleLeft, #14
        self.joints.append([0,0,0,14])  # footLeft, #15
        self.joints.append([0,0,0,0])   # hipRight, #16
        self.joints.append([0,0,0,16])  # kneeRight, #17
        self.joints.append([0,0,0,17])  # ankleRight, #18
        self.joints.append([0,0,0,18])  # footRight, #19
        self.joints.append([0,0,0,1])   # spineSoulder, #20
        self.joints.append([0,0,0,7])   # handTipLeft, #21
        self.joints.append([0,0,0,6])   # thumbLeft, #22
        self.joints.append([0,0,0,11])  # handTipRight, #23
        self.joints.append([0,0,0,10])  # thumbTipRight, #24

    # -----------------------------------------------------------------------------
    # canvas close event
    #
    def onclose(self, event):
        self.fig = None
        # if user closes this figure, let the main application know and to exit
        settings.application.close()

    #------------------------------------------------------------------------------
    # canvas resize event
    #
    def onresize(self, event):
        self.setAxisLimits()

    #------------------------------------------------------------------------------
    # slider update event
    #
    def onsliderupdate(self, val):
        # map to [0..1]
        p = self.slidertime.val / (self.slidertime.valmax - self.slidertime.valmin)
        self.selectTime(p, False)

        # call main application so that other graphs can be updated as well
        settings.application.selectTime(p, self)

    # -----------------------------------------------------------------------------
    #
    def updateInputName(self):
        self.fig.canvas.set_window_title(self.strTitle + ' - [' + settings.application.strBeautifiedInputFile + ']')

    #------------------------------------------------------------------------------
    #
    def saveView(self):
        if (self.fig is None):
            return

        filePath = os.path.join(settings.application.outputFolder, settings.application.outputName + '_3DJointData.png')
        filePath = settings.checkFileAlreadyExists(filePath, fileExt=".png", fileTypes=[('png files', '.png'), ('all files', '.*')])
        if (filePath is None):
            return

        try:
            self.fig.savefig(filePath, bbox_inches='tight')
            settings.application.logMessage("3D Joint Data view was saved to '" + settings.beautifyPath(filePath) + "'")
        except Exception as e:
            strError = e
            settings.application.logMessage("Exception saving 3D Joint Data view to '" + settings.beautifyPath(filePath) + "': " + str(e))

    #------------------------------------------------------------------------------
    # render frame blocks for both kinect and labanotation
    #
    def renderFrameBlocks(self):
        self.axFrameBlocks1.clear()
        self.axFrameBlocks2.clear()

        # after a clear() annSelection object is gone. reset variable
        self.annSelection = None

        padding = 0.0
        duration = 0.0

        cnt = len(self.jointFrames)
        if (cnt > 0):
            duration = self.jointFrames[cnt-1]['timeS'][0]

            maxTime = (duration / 1000.0)

            def format_func(value, tick_number):
                return r"${:.2f}$".format(value)

            self.axSliderTime.set_xlim((0, maxTime))
            self.axSliderTime.tick_params(axis='x', labelsize=8)
            self.axSliderTime.get_xaxis().set_major_locator(ticker.AutoLocator())
            self.axSliderTime.get_xaxis().set_minor_locator(ticker.AutoMinorLocator())

            # show minor ticks every 5 frames and major ticks every 10 frames on the top of frame block 1
            onetick = (maxTime) / float(cnt)
            self.axFrameBlocks1.set_xlim((0, maxTime))
            self.axFrameBlocks1.xaxis.tick_top()
            self.axFrameBlocks1.xaxis.set_minor_locator(ticker.MultipleLocator(onetick * 5))
            self.axFrameBlocks1.xaxis.set_major_locator(ticker.MultipleLocator(onetick * 10))
            self.axFrameBlocks1.set_xticklabels([])
            self.axFrameBlocks1.get_yaxis().set_visible(False)

            self.axFrameBlocks2.set_xlim((0, maxTime))
            self.axFrameBlocks2.get_xaxis().set_visible(False)
            self.axFrameBlocks2.get_yaxis().set_visible(False)

        # render individual kinect joint frame blocks
        xx = self.axFrameBlocks1.get_xlim()
        yy = self.axFrameBlocks1.get_ylim()
        cx = (xx[1] - xx[0])
        cy = (yy[1] - yy[0])
        padding = cx * 0.01
        cnt = len(self.jointFrames)
        if (cnt > 0):
            padding = (cx / cnt) * 0.3
            for i in range(0, cnt):
                start = (self.jointFrames[i]['timeS'][0] / 1000.0) - (padding / 2.0)
                x_width = padding
                isFilled = self.jointFrames[i]['filled'][0]
                c = 'r' if (isFilled) else 'b'
                p = patches.Rectangle((start, yy[0]), x_width, cy, alpha=0.50, color=c)
                self.axFrameBlocks1.add_patch(p)

        self.axFrameBlocks1.text((maxTime), (cy / 2.0), '  Original', horizontalalignment='left', verticalalignment='center', color='black')

        # render individual laban frame blocks
        xx = self.axFrameBlocks2.get_xlim()
        yy = self.axFrameBlocks2.get_ylim()
        cx = (xx[1] - xx[0])
        cy = (yy[1] - yy[0])
        cnt = len(self.timeS)
        if (cnt > 0):
            for i in range(0, cnt):
                start = (self.timeS[i] / 1000.0) - (padding / 2.0)
                x_width = padding
                p = patches.Rectangle((start, yy[0]), x_width, cy, alpha=0.50, color='g')
                self.axFrameBlocks2.add_patch(p)

        self.axFrameBlocks2.text((maxTime), (cy / 2.0), '  Labanotation', horizontalalignment='left', verticalalignment='center', color='black')

        self.fig.canvas.draw_idle()

    #------------------------------------------------------------------------------
    #
    def setJointFrames(self, jointFrames_in):
        self.jointFrames = copy.copy(jointFrames_in)

        if (len(self.jointFrames) > 0):
            timeS0 = self.jointFrames[len(self.jointFrames)-1]['timeS'][0]
            self.slidertime.valmax = float(timeS0) / 1000.0
        else:
            self.slidertime.valmax = 0.0

        # set the slider axes to take valmax change
        self.slidertime.ax.set_xlim(self.slidertime.valmin, self.slidertime.valmax)
        self.fig.canvas.draw_idle()

        self.renderFrameBlocks()

    #------------------------------------------------------------------------------
    #
    def setLabanotation(self, timeS, all_laban):
        self.timeS = copy.copy(timeS)
        self.all_laban = copy.copy(all_laban)

        self.renderFrameBlocks()

    #------------------------------------------------------------------------------
    #
    def selectTime(self, time, fUpdateSlider=False):
        time = self.slidertime.valmin + (time * (self.slidertime.valmax - self.slidertime.valmin))

        # disconnect slider update callback to avoid endless loop. Reconnect 
        # once slider value reset
        if (fUpdateSlider):
            self.slidertime.disconnect(self.cidSlider)
            self.slidertime.set_val(time)
            self.cidSlider = self.slidertime.on_changed(self.onsliderupdate)

        cnt = len(self.jointFrames)
        if (cnt == 0):
            return

        # find the frame corresponding to the given time
        for idx in range(0,cnt):
            temp = self.jointFrames[idx]
            kt = float(temp['timeS'][0]) / 1000.0
            if (kt >= time):
                break

            self.isInterpolatedKeyFrame = temp['filled'][0]

        # take a kinect snapshot of joints in time and render them.
        # Map to graph's xyz space
        a = self.joints
        for i in range(0, 25):
            a[i][0] = -temp[0][i+2][2]
            a[i][1] = -temp[0][i+2][0]
            a[i][2] =  temp[0][i+2][1]

        self.drawKinectSkeleton()
 
        if ((self.all_laban is not None) and (len(self.all_laban) > 0)):
            cnt = len(self.timeS)

            # find the frame corresponding to the given time
            for idx in range(0, cnt):
                kt = (self.timeS[idx]) / 1000.0
                if (kt >= time):
                    break

            laban = self.all_laban[idx]
            time = int(self.timeS[idx])

            if (settings.fVerbose):
                print('Right Elbow:' + str(elR[t][1]) + ', ' + str(elR[t][2]))
                print('Right Wrist:' + str(wrR[t][1]) + ', ' + str(wrR[t][2]))
                print('Left Elbow:' + str(elL[t][1]) + ', ' + str(elL[t][2]))
                print('Left Wrist:' + str(wrL[t][1]) + ', ' + str(wrL[t][2]))

            curr_laban = ['Start Time:' + str(time), 'Duration:0', 'Head:Forward:Normal', 
                'Right Elbow:' + laban[0][0] + ':' + laban[0][1], 'Right Wrist:' + laban[1][0] + ':' + laban[1][1], 
                'Left Elbow:' + laban[2][0] + ':' + laban[2][1], 'Left Wrist:' + laban[3][0] + ':' + laban[3][1], 
                'Rotation:ToLeft:0.0']

            self.drawLabanotationSkeleton(laban = curr_laban)

            if (self.axFrameBlocks2 != None):
                if (self.annSelection != None):
                     self.annSelection.remove()

                color = 'green'
                time = time / 1000.0
                self.annSelection = self.axFrameBlocks2.annotate('', xy=(time, 0.0), xytext=(time, -0.5),
                    weight='bold', color=color,
                    arrowprops=dict(arrowstyle='wedge', connectionstyle="arc3", color=color))

    #------------------------------------------------------------------------------
    #
    def setAxisLimits(self):
        # get canvas size in pixels
        size = self.fig.get_size_inches() * self.fig.dpi

        # calculate axis limits while keeping aspect ratio at 1
        aspect = size[0] / size[1];
        min = -20
        max = 20
        axmin = (min) if (aspect < 1) else (min * aspect)
        axmax = (max) if (aspect < 1) else (max * aspect)
        aymin = (min) if (aspect > 1) else (min / aspect)
        aymax = (max) if (aspect > 1) else (max / aspect)

        # set limits. Remember our axis mapped (x=z), (y=x) and (z=y)
        self.ax.set_xlim3d(min, max)
        self.ax.set_ylim3d(axmin, axmax)
        self.ax.set_zlim3d(aymin, aymax)

    #------------------------------------------------------------------------------
    #
    def resetGraph(self):
        # clear canvas
        self.ax.clear()

        self.setAxisLimits()
        
        # Draw x, y, and z axis markers in the same way you were in
        # the code snippet in your question...
        xspan, yspan, zspan = 3 * [np.linspace(0, 6, 15)]
        zero = np.zeros_like(xspan)

        self.ax.plot3D(xspan, zero, zero, 'k--', gid='axis', color='black')
        self.ax.plot3D(zero, yspan, zero, 'k--', gid='axis', color='black')
        self.ax.plot3D(zero, zero, zspan, 'k--', gid='axis', color='black')

        q = -.20
        w = 1
        self.ax.text(xspan.max() + w, q, q, "z", gid='axis', color='black')
        self.ax.text(q, yspan.max() + w, q, "x", gid='axis', color='black')
        self.ax.text(q, q, zspan.max() + w, "y", gid='axis', color='black')

        self.ax.xaxis._axinfo['tick']['color'] = (1.0, 1.0, 1.0, 0.5)
        self.ax.yaxis._axinfo['tick']['color'] = (1.0, 1.0, 1.0, 0.5)
        self.ax.zaxis._axinfo['tick']['color'] = (1.0, 1.0, 1.0, 0.5)

        self.ax.xaxis._axinfo['axisline']['color'] = (1.0, 1.0, 1.0, 0.5)
        self.ax.yaxis._axinfo['axisline']['color'] = (1.0, 1.0, 1.0, 0.5)
        self.ax.zaxis._axinfo['axisline']['color'] = (1.0, 1.0, 1.0, 0.5)

        # get rid of the panes                          
        self.ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0)) 
        self.ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 0.5)) 
        # self.ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0)) 

        # get rid of the spines                         
        #self.ax.xaxis.line.set_color((1.0, 1.0, 1.0, 0.0)) 
        #self.ax.yaxis.line.set_color((1.0, 1.0, 1.0, 0.5)) 
        #self.ax.zaxis.line.set_color((1.0, 1.0, 1.0, 0.5))

        # don't show tick labels
        self.ax.xaxis.set_major_formatter(plt.NullFormatter())
        self.ax.yaxis.set_major_formatter(plt.NullFormatter())
        self.ax.zaxis.set_major_formatter(plt.NullFormatter())

    #------------------------------------------------------------------------------
    #
    #   rotate around x axis:   rotate around y axis:   rotates around
    #    np.array([              np.array([              np.array([
    #        [1,  0,  0],            [c,  0, -s],            [c, -s,  0],
    #        [0,  c, -s],            [0,  1,  0],            [s,  c,  0],
    #        [0,  s,  c]             [s,  0,  c]             [0,  0,  1],
    #      ]))                     ]))                     ]))

    def correctSkeletonRotation(self):
        shL = np.zeros(3)
        shR = np.zeros(3)
        spM = np.zeros(3)
        shL[0] = self.joints[4][1]  # left shoulder
        shL[1] = self.joints[4][2]
        shL[2] = self.joints[4][0]
        shR[0] = self.joints[8][1]  # right shoulder
        shR[1] = self.joints[8][2]
        shR[2] = self.joints[8][0]
        spM[0] = self.joints[1][1]  # spine middle
        spM[1] = self.joints[1][2]
        spM[2] = self.joints[1][0]
        
        # find where the performer is facing, rotate the body figure
        sh = np.zeros((3,3))
        v1 = shL-shR
        v2 = spM-shR
        sh[0] = np.cross(v2,v1) # x axis
        x = sh[0][0]
        y = sh[0][1]
        z = sh[0][2]

        # rotate around y axis
        r = math.sqrt(z*z+x*x)
        sinth = x/r
        costh = z/r
        conv = np.zeros((3,3))
        conv[0][0] = costh
        conv[0][1] = 0
        conv[0][2] = -sinth
        conv[1][0] = 0
        conv[1][1] = 1
        conv[1][2] = 0
        conv[2][0] = sinth
        conv[2][1] = 0
        conv[2][2] = costh
        
        for i in range(len(self.joints)):
            tmp_in = np.zeros(3)
            tmp_in[0] = self.joints[i][1]
            tmp_in[1] = self.joints[i][2]
            tmp_in[2] = self.joints[i][0]
            tmp_out = np.dot(conv,tmp_in)
            self.joints[i][1] = tmp_out[0]
            self.joints[i][2] = tmp_out[1]
            self.joints[i][0] = tmp_out[2]


    #------------------------------------------------------------------------------
    #
    def draw_joints(self, c, a = 1.0):
        alpha = a
        cnt = len(self.joints) - 4 # do not draw hand tips and thumbs
        for i in range(cnt):
            if (i == 7) or (i == 11): # do not draw hands
                continue

            if (i==20) or (i==4) or (i==5) or (i==6) or (i==8) or (i==9) or (i==10):
                alpha = a
            else:
                alpha = 0.3

            p = self.joints[i]
            self.ax.scatter(p[0],p[1],p[2],color=c, alpha=alpha)

    #------------------------------------------------------------------------------
    #
    def draw_limbs(self):
        for i in range(1, len(self.joints) - 4):
            if (i == 7) or (i == 11): # do not draw hands
                continue

            if (i==4) or (i==5) or (i==6) or (i==8) or (i==9) or (i==10):
                alpha = 1.0
            else:
                alpha = 0.3

            a = self.joints[i]
            start = [a[0],a[1],a[2]]
            b = self.joints[self.joints[i][3]]
            end = [b[0],b[1],b[2]]
            self.ax.plot([start[0],end[0]],[start[1],end[1]],[start[2],end[2]],'--', color='k', alpha=alpha, gid='limbs')

        # body frame
        a = self.joints[1]
        start = [a[0],a[1],a[2]]
        b = self.joints[4]
        end = [b[0],b[1],b[2]]
        self.ax.plot([start[0],end[0]],[start[1],end[1]],[start[2],end[2]],'--', color='k', alpha=0.3, gid='limbs')
        b = self.joints[8]
        end = [b[0],b[1],b[2]]
        self.ax.plot([start[0],end[0]],[start[1],end[1]],[start[2],end[2]],'--', color='k', alpha=0.3, gid='limbs')
    
    #------------------------------------------------------------------------------
    #
    def drawKinectSkeleton(self):
        self.resetGraph()

        # center skeleton around its spineB
        move_0 = self.joints[0][0]
        move_1 = self.joints[0][1]
        move_2 = self.joints[0][2]

        for i in range(0,len(self.joints)):
            self.joints[i][0] = self.joints[i][0]-move_0
            self.joints[i][1] = self.joints[i][1]-move_1
            self.joints[i][2] = self.joints[i][2]-move_2

        # rotate skeleton to face forward towards camera
        self.correctSkeletonRotation()

        # scale, spine_shoulder->spine_midlle is 5
        d0 = self.joints[20][0]-self.joints[1][0]
        d1 = self.joints[20][1]-self.joints[1][1]
        d2 = self.joints[20][2]-self.joints[1][2]

        d = (d0**2+d1**2+d2**2)**0.5

        self.scale = (5.0 / d)

        for i in range(0, len(self.joints)):
            self.joints[i][0] = (self.joints[i][0] * self.scale)
            self.joints[i][1] = (self.joints[i][1] * self.scale) - 15      # offset from center
            self.joints[i][2] = (self.joints[i][2] * self.scale)
            
        faceColor = 'r' if self.isInterpolatedKeyFrame else 'b'

        self.draw_joints(faceColor, 0.50)
        self.draw_limbs()

        # skeleton title
        self.ax.text(self.joints[3][0], self.joints[3][1], self.joints[3][2] + 2, 'Original',
                bbox={'facecolor':faceColor,'alpha':0.50,'edgecolor':'gray','pad':3.5},
                ha='center', va='bottom', color='w') 

        self.fig.canvas.draw_idle()

    #------------------------------------------------------------------------------
    # convert labanotation to joint points
    #
    def mapLabanotation2Joints(self, laban):
        self.calc_joint(1,  [0, 6, 0])                  # spineBase->spineMid
        self.calc_joint(20, [0, 5, 0])                  # spineMid->spineShoulder
        self.calc_joint(2,  [0, 2, 0])                  # spineShoulder->neck
        self.calc_joint(3,  [0, 3, 0])                  # neck->head
        self.calc_joint(4,  [4,-0.5,0])                 # spineShoulder->shoulderLeft

        vec = self.laban2vec(laban, "left elbow")       # shoulderLeft->elbowLeft
        self.calc_joint(5,vec)

        vec = self.laban2vec(laban, "left wrist")       # elbowLeft->wristLeft
        self.calc_joint(6,vec)

        self.calc_joint(7, [0,0,0])                     # wristLeft->handLeft
        self.calc_joint(8, [-4,-0.5,0])                 # spineShoulder->shoulderRight

        vec = self.laban2vec(laban, "right elbow")      # shoulderRight->elbowRight
        self.calc_joint(9,vec)

        vec = self.laban2vec(laban, "right wrist")      # elbowRight->wristRight
        self.calc_joint(10,vec)

        self.calc_joint(11, [0,0,0])                    # wristLeft->handLeft
        self.calc_joint(12, [1.5,-0, 0.5])              # spineBase->hipLeft
        self.calc_joint(13, [0,-6,0])                   # hipLeft->kneeLeft
        self.calc_joint(14, [0,-7,0])                   # kneeLeft->ankleLeft
        self.calc_joint(15, [0.2,-0.2,3])               # ankleLeft->footLeft
        self.calc_joint(16, [-1.5,-0, 0.5])             # spineBase->hipRight
        self.calc_joint(17, [0,-6,0])                   # hipRight->kneeRight
        self.calc_joint(18, [0,-7,0])                   # kneeRight->ankleRight
        self.calc_joint(19, [-0.2,-0.2,3])              # ankleRight->footRight

    #------------------------------------------------------------------------------
    # calculate the given joint using its parent joint and a vector
    #
    def calc_joint(self, target, vec):
        self.joints[target][0] = self.joints[self.joints[target][3]][0] + vec[0]
        self.joints[target][1] = self.joints[self.joints[target][3]][1] + vec[1]
        self.joints[target][2] = self.joints[self.joints[target][3]][2] + vec[2]

    #------------------------------------------------------------------------------
    # convert the labanotation for a given limb to a vector
    #
    def laban2vec(self, laban, limb):
        theta = 175
        phi = 0
        pi = 3.1415926
        for i in range(0, len(laban)):
            laban[i] = laban[i].lower()
            tmp_str = laban[i].split(":")
            if tmp_str[0]==limb:
                lv = tmp_str[2]
                if lv == "high":
                    theta = 45
                elif lv == "normal":
                    theta = 90
                elif lv == "low":
                    theta = 135
                else:
                    theta = 180
                    print('Unknown Level.')
                    
                dire = tmp_str[1]
                if dire == "forward":
                    phi = 0
                elif dire == "right forward":
                    phi = -45
                elif dire == "right":
                    phi = -90
                elif dire == "right backward":
                    phi = -135
                elif dire == "backward":
                    phi = 180
                elif dire == "left backward":
                    phi = 135
                elif dire == "left":
                    phi = 90
                elif dire == "left forward":
                    phi = 45
                elif dire == "place":
                    if lv == "high":
                        theta = 5
                        phi = 0
                    elif lv == "low":
                        theta = 175
                        phi = 0
                    else:
                        theta = 180
                        phi = 0
                        print('Unknown Place')
                else:
                    phi = 0
                    print('Unknown Direction.')
                break

        # length of the limb
        l = 5
        y = l * math.cos(theta/180.0*pi)
        x = l * math.sin(theta/180.0*pi) * math.sin(phi/180.0*pi)
        z = l * math.sin(theta/180.0*pi) * math.cos(phi/180.0*pi)

        return [x,y,z]

    #------------------------------------------------------------------------------
    #
    def drawLabanotationSkeleton(self, laban=""):
        if (laban == ''):
            return

        for i in range(0,len(self.joints)):
            self.joints[i][0] = 0
            self.joints[i][1] = 0
            self.joints[i][2] = 0

        self.mapLabanotation2Joints(laban)

        # map to graph xyz
        for i in range(0,len(self.joints)):
            x = self.joints[i][0]
            y = self.joints[i][1]
            z = self.joints[i][2]

            self.joints[i][0] = z
            self.joints[i][1] = x + 15      # offset from center
            self.joints[i][2] = y

        self.draw_joints('g', 0.50)
        self.draw_limbs()

        # skeleton title
        self.ax.text(self.joints[3][0], self.joints[3][1], self.joints[3][2] + 2, 'Labanotation',
                bbox={'facecolor':'g','alpha':0.50,'edgecolor':'gray','pad':3.5},
                ha='center', va='bottom', color='w') 
        
        self.fig.canvas.draw_idle()

    #------------------------------------------------------------------------------
    #
