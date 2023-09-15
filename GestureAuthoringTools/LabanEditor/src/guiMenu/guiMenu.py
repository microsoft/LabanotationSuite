# --------------------------------------------------------------------------------------------
# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.
# --------------------------------------------------------------------------------------------

import sys, os, math
from math import sin, cos, sqrt, radians
import pickle
import numpy as np
from operator import sub

#import Tkinter, tkFileDialog
#from Tkinter import *
import _tkinter
from _tkinter import *

from tkinter import filedialog

import matplotlib
import matplotlib.pyplot as plt
plt.rcParams['toolbar'] = 'None'
plt.rcParams.update( {'font.size': 8 })
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backend_tools import ToolBase, ToolToggleBase
from matplotlib.widgets import Slider, Button, RadioButtons, CheckButtons
from mpl_toolkits.axes_grid1.inset_locator import InsetPosition
from matplotlib import gridspec
import matplotlib.ticker as ticker

import labanotation.tool.wavfilter as wf

import settings

class CaptureOutput:
    def write(self, message):
        settings.application.logMessage(message, ioRedirect=True)


class guiMenu:
    fig = None

    gauss_params = [33, 7]

    logTextLabels = []
    logText = []

    axMisc = None
    axAlgorithm = None
    axSaveOptions = None
    axGauss = None
    axLog = None
    axRadio = None
    axLogLabels = None

    line_gaussian = None

    ax_window_size = None
    slider_window_size = None

    ax_sigma = None
    slider_sigma = None

    originalStdOut = None

    #------------------------------------------------------------------------------
    # Class initialization
    #
    def __init__(self):
        self.strTitle = 'Gesture Analysis Configuration - ' + settings.appVersion
        self.fig = plt.figure()
        self.fig.canvas.set_window_title(self.strTitle)
        self.fig.set_size_inches((settings.screen_cx * 0.49) / self.fig.dpi, (settings.screen_cy * 0.465) / self.fig.dpi)

        self.fig.canvas.mpl_connect('resize_event', self.onresize)
        self.fig.canvas.mpl_connect('close_event', self.onclose)

        self.fig.edgecolor = 'blue'
        self.fig.set_facecolor('0.90')

        left  = 0.03    # the left side of the subplots of the figure
        right = 0.97    # the right side of the subplots of the figure
        bottom = 0.04   # the bottom of the subplots of the figure
        top = 0.92      # the top of the subplots of the figure
        wspace = 0.3    # the amount of width reserved for blank space between subplots
        hspace = 0.7    # the amount of height reserved for white space between subplots

        plt.subplots_adjust(left=left, top=top, right=right, bottom=bottom, wspace=wspace, hspace=hspace)

        self.axMisc = plt.subplot2grid((6, 4), (0, 0), rowspan=1, colspan=1, aspect='auto', anchor='NW')
        self.axAlgorithm = plt.subplot2grid((6, 4), (1, 0), rowspan=2, colspan=1, aspect='auto', anchor='NW')
        self.axSaveOptions = plt.subplot2grid((6, 4), (3, 0), rowspan=3, colspan=1, aspect='equal', anchor='NW')
        self.axGauss = plt.subplot2grid((6, 4), (0, 1), rowspan=4, colspan=3)
        self.axLog = plt.subplot2grid((6, 4), (5, 1), rowspan=1, colspan=3, aspect='auto', anchor='NW')

        # create the various groups of UI controls
        self.createUIMiscellaneous()
        self.createUILog()

        self.createUIAlgorithm()
        self.createUIGaussianFilterControls()
        self.createUIMiscellaneousControls()

        # set settings.tkGuiCanvas for use for modal dialogs
        try:
            if (self.fig is not None) and (self.fig.canvas is not None) and (self.fig.canvas._tkcanvas is not None):
                settings.tkGuiCanvas = self.fig.canvas._tkcanvas
        except Exception as e:
            pass

    #------------------------------------------------------------------------------
    #
    def get_aspect(self, ax):
        # Total figure size
        figW, figH = ax.get_figure().get_size_inches()
        # Axis size on figure
        _, _, w, h = ax.get_position().bounds
        # Ratio of display units
        disp_ratio = (figH * h) / (figW * w)
        # Ratio of data units
        # Negative over negative because of the order of subtraction
        data_ratio = sub(*ax.get_ylim()) / sub(*ax.get_xlim())

        return disp_ratio / data_ratio

    #------------------------------------------------------------------------------
    #
    def createUIAlgorithm(self):
        algorithm = settings.application.algorithm.lower()
        defaultAlgoritmIdx = 0
        if (algorithm == 'total'):
            defaultAlgoritmIdx = 0
        elif (algorithm == 'parallel'):
            defaultAlgoritmIdx = 1
        elif (algorithm == 'naive'):
            defaultAlgoritmIdx = 2

        self.axAlgorithm.set_title('Algorithm', x=0, horizontalalignment='left')

        # create an axis to host to radio buttons. make its aspect ratio
        # equal so the radiobuttons stay round
        aspect = self.get_aspect(self.axAlgorithm)
        rect = [0, 0, 1.0 * aspect, 1.0]
        ip = InsetPosition(self.axAlgorithm, rect)
        self.axRadio = plt.axes(rect)
        self.axRadio.set_axes_locator(ip)
        self.axRadio.axis('off')

        self.radioAlgorithm = RadioButtons(self.axRadio, ('Total Energy', 'Parallel Energy', 'Naive (no filter)'), active=defaultAlgoritmIdx)
        self.radioAlgorithm.on_clicked(self.onClickAlgorithm)

    #------------------------------------------------------------------------------
    #
    def createUIGaussianFilterControls(self):
        axcolor = 'lightgoldenrodyellow'

        rect = [0.82, -0.158, 0.14, 0.07]
        ax_btnapply = plt.axes(rect)
        ip = InsetPosition(self.axGauss, rect) #posx, posy, width, height
        ax_btnapply.set_axes_locator(ip)
        self.btnApply = Button(ax_btnapply, 'Apply')
        self.btnApply.on_clicked(self.onclickApply)

        rect = [0.82, -0.245, 0.14, 0.07]
        ax_btnreset = plt.axes(rect)
        ip = InsetPosition(self.axGauss, rect) #posx, posy, width, height
        ax_btnreset.set_axes_locator(ip)
        self.btnReset = Button(ax_btnreset, 'Reset', color='0.950', hovercolor='0.975')
        self.btnReset.on_clicked(self.onclickReset)

        rect = [0.1, -0.155, 0.55, 0.04]
        self.ax_window_size = plt.axes(rect, facecolor=axcolor)
        ip = InsetPosition(self.axGauss, rect) #posx, posy, width, height
        self.ax_window_size.set_axes_locator(ip)
        self.slider_window_size = Slider(self.ax_window_size, 'Window Size', 1, (self.gauss_params[0]+1)*2 + 1, valinit=self.gauss_params[0], valstep=2)
        self.slider_window_size.on_changed(self.updateGaussianFilter)

        rect = [0.1, -0.235, 0.55, 0.04]
        self.ax_sigma = plt.axes(rect, facecolor=axcolor)
        ip = InsetPosition(self.axGauss, rect) #posx, posy, width, height
        self.ax_sigma.set_axes_locator(ip)
        self.slider_sigma = Slider(self.ax_sigma, 'Sigma', 1, (self.gauss_params[1]+1)*2, valinit=self.gauss_params[1], valstep=1)
        self.slider_sigma.on_changed(self.updateGaussianFilter)

        self.updateGaussianFilter()

    #------------------------------------------------------------------------------
    #
    def createUIMiscellaneous(self):
        self.axMisc.set_title('', x=0, horizontalalignment='left')

        # removing top and right borders
        self.axMisc.xaxis.set_visible(False)
        self.axMisc.yaxis.set_visible(False)

        # remove ticks
        self.axSaveOptions.set_xticks([])
        self.axSaveOptions.set_yticks([])

        # remove ticks
        self.axAlgorithm.set_xticks([])
        self.axAlgorithm.set_yticks([])

        bbox = self.axMisc.get_window_extent()
        self.axMisc.set_xlim(0, bbox.width)
        self.axMisc.set_ylim(0, bbox.height)

    #------------------------------------------------------------------------------
    #
    def createUILog(self):
        self.axLog.set_title('Console Log', x=0, horizontalalignment='left')

        # removing top and right borders
        self.axLog.xaxis.set_visible(False)
        self.axLog.yaxis.set_visible(False)

        self.resetLogLabels()

        # redirect console messages to gui's log
        self.originalStdOut = sys.stdout
        sys.stdout = CaptureOutput()

    # -----------------------------------------------------------------------------
    #
    def resetLogLabels(self):
        cnt = len(self.logTextLabels)
        for i in range(0, cnt):
            self.logTextLabels[i].remove()

        self.logTextLabels = []

        bbox = self.axLog.get_window_extent()
        self.axLog.set_xlim(0, bbox.width)
        self.axLog.set_ylim(0, bbox.height)

        aspect = self.get_aspect(self.axLog)
        rect = [0, 0, 1.0 * aspect, 1.0]
        ip = InsetPosition(self.axLog, rect)

        if (self.axLogLabels is None):
            self.axLogLabels = plt.axes(rect)
        else:
            self.axLogLabels.set_position(rect)

        self.axLogLabels.set_axes_locator(ip)
        self.axLogLabels.axis('off')

        aspectLog = 1.0 / self.get_aspect(self.axLog)
        strText = 'Tyg'
        tmp, self.logTextHeight = settings.getTextExtent(self.axLog, strText)
        self.logTextHeight = self.logTextHeight * aspectLog  # * self.fig.dpi

        # pre-create empty log label placeholders
        self.logTextLabels = []
        y = (self.logTextHeight / 4.0)
        cy = bbox.height
        idx = len(self.logText) - 1
        while (y < cy):
            str = self.logText[idx] if (idx >= 0) else ''
            idx = idx - 1

            lbl = self.axLogLabels.text(8.0, y, str, horizontalalignment='left', verticalalignment='bottom', color='dimgray', clip_on=True, transform=self.axLog.transData) #, bbox={'facecolor':'lightgray', 'alpha':0.7, 'pad':0.0})

            self.logTextLabels.append(lbl)
            y += self.logTextHeight

    # -----------------------------------------------------------------------------
    #
    def createUIMiscellaneousControls(self):
        rect = [0.06, 0.30, 0.70, 0.40]
        ip = InsetPosition(self.axMisc, rect) #posx, posy, width, height
        ax_btnbrowse = plt.axes(rect)
        ax_btnbrowse.set_axes_locator(ip)
        self.btnBrowse = Button(ax_btnbrowse, 'Input File')
        self.btnBrowse.on_clicked(self.onclickBrowse)

        self.axSaveOptions.set_title('Output Files', x=0, horizontalalignment='left')

        x = 0.06
        dx = 0.70 # 0.80
        dy = 0.10 # 0.17
        cy = 0.14 # 0.24
        y = 0.80
        rect = [x, y, dx, dy]
        ip = InsetPosition(self.axSaveOptions, rect) #posx, posy, width, height
        ax_btn = plt.axes(rect)
        ax_btn.set_axes_locator(ip)
        self.btnSaveJSON = Button(ax_btn, 'JSON')
        self.btnSaveJSON.on_clicked(self.onclickSaveJSON)

        rect = [x, y - 1*cy, dx, dy]
        ip = InsetPosition(self.axSaveOptions, rect) #posx, posy, width, height
        ax_btn = plt.axes(rect)
        ax_btn.set_axes_locator(ip)
        self.btnSaveTXT = Button(ax_btn, 'Text')
        self.btnSaveTXT.on_clicked(self.onclickSaveTXT)

        rect = [x, y - 2*cy, dx, dy]
        ip = InsetPosition(self.axSaveOptions, rect) #posx, posy, width, height
        ax_btn = plt.axes(rect)
        ax_btn.set_axes_locator(ip)
        self.btnSaveFilterView = Button(ax_btn, 'Filter Graph')
        self.btnSaveFilterView.on_clicked(self.onclickSaveFilterView)

        rect = [x, y - 3*cy, dx, dy]
        ip = InsetPosition(self.axSaveOptions, rect) #posx, posy, width, height
        ax_btn = plt.axes(rect)
        ax_btn.set_axes_locator(ip)
        self.btnSaveSkeletonView = Button(ax_btn, '3D Joint Data')
        self.btnSaveSkeletonView.on_clicked(self.onclickSaveSkeletonView)

        rect = [x, y - 4*cy, dx, dy]
        ip = InsetPosition(self.axSaveOptions, rect) #posx, posy, width, height
        ax_btn = plt.axes(rect)
        ax_btn.set_axes_locator(ip)
        self.btnSaveScoreView = Button(ax_btn, 'Score View')
        self.btnSaveScoreView.on_clicked(self.onclickSaveScoreView)

        rect = [x, y - 5*cy, dx, dy]
        ip = InsetPosition(self.axSaveOptions, rect) #posx, posy, width, height
        ax_btn = plt.axes(rect)
        ax_btn.set_axes_locator(ip)
        self.btnSavePNG = Button(ax_btn, 'Full Score')
        self.btnSavePNG.on_clicked(self.onclickSaveImage)

    #------------------------------------------------------------------------------
    # canvas resize event
    #
    def onresize(self, event):
        # plt.tight_layout()

        # keep tha radio buttons round...
        if (self.axRadio is not None) and (self.axAlgorithm is not None):
            aspect = self.get_aspect(self.axAlgorithm)
            rect = [0, 0, 1.0 * aspect, 1.0]

            ip = InsetPosition(self.axAlgorithm, rect)
            self.axRadio.set_axes_locator(ip)
            self.axRadio.set_position(rect)

        self.resetLogLabels()


    # -----------------------------------------------------------------------------
    # canvas close event
    #
    def onclose(self, event):
        self.fig = None
        # if user closes this figure, let the main application know and to exit
        settings.application.close()

    # -----------------------------------------------------------------------------
    #
    def updateUIControls(self, algorithm):
        algorithm = algorithm.lower()
        fEnable = False if (algorithm == 'naive') else True
        alpha = 0.2 if (algorithm == 'naive') else 1.0

        self.btnApply.set_active(fEnable)
        self.btnReset.set_active(fEnable)

        self.btnApply.label.set_alpha(alpha) 
        self.btnReset.label.set_alpha(alpha) 

        fUpdateGaussianPlot = False
        if (self.gauss_params[0] != self.slider_window_size.val):
            self.ax_window_size.clear()
            self.slider_window_size.__init__(self.ax_window_size, 'Window Size', valmin=1, valmax=(self.gauss_params[0]+1)*2 + 1, valinit=self.gauss_params[0], valstep=2)
            self.slider_window_size.on_changed(self.updateGaussianFilter)
            fUpdateGaussianPlot = True

        if (self.gauss_params[1] != self.slider_sigma.val):
            self.ax_sigma.clear()
            self.slider_sigma.__init__(self.ax_sigma, 'Sigma', valmin=1, valmax=(self.gauss_params[1]+1)*2, valinit=self.gauss_params[1], valstep=1)
            self.slider_sigma.on_changed(self.updateGaussianFilter)
            fUpdateGaussianPlot = True

        if (fUpdateGaussianPlot):
            self.updateGaussianFilter()

        self.line_gaussian.set_alpha(alpha) 

        self.ax_window_size.patch.set_alpha(alpha)
        if (self.slider_window_size.poly):
            self.slider_window_size.poly.set_alpha(alpha)
        self.slider_window_size.set_active(fEnable)
        for r in self.slider_window_size.ax.texts:
            r.set_alpha(alpha) 

        self.ax_sigma.patch.set_alpha(alpha)
        if (self.slider_sigma.poly):
            self.slider_sigma.poly.set_alpha(alpha)
        self.slider_sigma.set_active(fEnable)
        for r in self.slider_sigma.ax.texts:
            r.set_alpha(alpha) 

        self.fig.canvas.draw_idle()

    # -----------------------------------------------------------------------------
    #
    def updateInputName(self):
        self.fig.canvas.set_window_title(self.strTitle + ' - [' + settings.application.strBeautifiedInputFile + ']')

    # -----------------------------------------------------------------------------
    #
    def getAlgorithmSelection(self):
        if (self.radioAlgorithm.value_selected == 'Total Energy'):
            return 'Total'
        elif (self.radioAlgorithm.value_selected == 'Parallel Energy'):
            return 'Parallel'

        return 'Naive'

    # -----------------------------------------------------------------------------
    #
    def onClickAlgorithm(self, label):
        algorithm = self.getAlgorithmSelection()

        if (settings.application.labanotation is not None):
            self.gauss_params = settings.application.labanotation.getGaussianParameters(algorithm)

        self.updateUIControls(algorithm)
        settings.application.applyAlgoritm(algorithm)

    #------------------------------------------------------------------------------
    # updateGaussianFilter() has an unused parameter, though needs it because the 
    # sliders use this function as their update callback...
    def updateGaussianFilter(self, val=0):
        # remove current gaussian lines
        if (self.line_gaussian is not None):
            self.line_gaussian.remove()
            del self.line_gaussian
            self.line_gaussian = None

        gauss_params = (int(self.slider_window_size.val), int(self.slider_sigma.val))

        self._t = np.arange(-gauss_params[0] / 2.0 + 0.5, gauss_params[0] / 2.0 + 0.5, 1.0)
        s = wf.gaussFilter(gauss_params[0], gauss_params[1])
        self.line_gaussian, = self.axGauss.plot(self._t, s, marker="o", linestyle='-', color='red', lw=1)

        # for i, txt in enumerate(s):
        #    self.axGauss.annotate("{:0.2f}".format(txt), (self._t[i], s[i]))

        wnd = int(gauss_params[0] / 2) + 1
        self.axGauss.set_xlim(-wnd, wnd)
        self.axGauss.set_ylim(0, 0.42) # np.max(s))
        self.fig.canvas.draw_idle()

    #------------------------------------------------------------------------------
    #
    def onclickApply(self, event):
        algorithm = self.getAlgorithmSelection()

        self.gauss_params = (int(self.slider_window_size.val), int(self.slider_sigma.val))

        if (settings.application.labanotation is not None):
            settings.application.labanotation.setGaussianParameters(algorithm, self.gauss_params)

        settings.application.applyAlgoritm(algorithm)

    #------------------------------------------------------------------------------
    #
    def onclickSaveJSON(self, event):
        settings.application.saveJSON()

    #------------------------------------------------------------------------------
    #
    def onclickSaveTXT(self, event):
        settings.application.saveTXT()

    #------------------------------------------------------------------------------
    #
    def onclickSaveImage(self, event):
        settings.application.saveImage()

    #------------------------------------------------------------------------------
    #
    def onclickSaveFilterView(self, event):
        settings.application.saveFilterView()

    #------------------------------------------------------------------------------
    #
    def onclickSaveSkeletonView(self, event):
        settings.application.saveSkeletonView()

    #------------------------------------------------------------------------------
    #
    def onclickSaveScoreView(self, event):
        settings.application.saveScoreView()

    #------------------------------------------------------------------------------
    #
    def onclickReset(self, event):
        self.slider_window_size.reset()
        self.slider_sigma.reset() 

    #------------------------------------------------------------------------------
    #
    def onclickBrowse(self, event):
        file = self.selectInputFile()
        if (file is None):
            return

        settings.application.openAndProcessInputfile(file)

    #------------------------------------------------------------------------------
    #
    def selectInputFile(self):
        fTyp = [("Kinect Joint Data File", "*.csv")]

        splitInput = os.path.split(os.path.abspath(settings.application.inputFilePath))

        options = {}
        options['filetypes'] = fTyp
        options['initialdir'] = splitInput[0].replace('/', os.sep)

        if (settings.tkGuiCanvas is not None):
            options['parent'] = settings.tkGuiCanvas

        file = filedialog.askopenfilename(**options)

        if not file:
            return None

        return file

    #------------------------------------------------------------------------------
    #
    def logMessage(self, str, ioRedirect=False):
        # also write message to console
        if (self.originalStdOut is not None):
            extra = "\r\n" if (ioRedirect is False) else ""
            self.originalStdOut.write(str + extra)

        self.logText.append(str)
        cnt = len(self.logTextLabels)
        if (cnt > 0):
            for i in range(cnt-1, 0, -1):
                self.logTextLabels[i].set_text(self.logTextLabels[i-1].get_text())

            self.logTextLabels[0].set_text(str)

            self.fig.canvas.draw_idle()

