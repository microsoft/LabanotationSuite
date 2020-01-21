# --------------------------------------------------------------------------------------------
# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.
# --------------------------------------------------------------------------------------------

import os

import matplotlib.pyplot as plt
plt.rcParams['toolbar'] = 'None'
from matplotlib.pyplot import figure, show
from matplotlib.widgets import Slider, Button, RadioButtons
import matplotlib.cm as cm
import matplotlib.patches as patches

import tkMessageBox

import cv2

import settings
import scrollbar

class graphLaban:
    fig = None
    ax = None
    im = None
    view = None
    imgWidth = 0
    imgHeight = 0
    axesAspect = 1.0
    axesHeight = 0.0
    drag_active = False
    x0 = None
    y0 = None
    press = None
    timeOffset = 0
    selectedFrame = 0
    selectedFrameMarker = None
    currentTime = 0

    #------------------------------------------------------------------------------
    # Class initialization
    #
    def __init__(self):
        self.strTitle = 'Labanotation Score'
        self.fig = figure()

        nc = 20
        self.ax = plt.subplot2grid((1, nc), (0, 0), rowspan=1, colspan=(nc-1), aspect=1, anchor='E')
        self.axSlider = plt.subplot2grid((1, nc), (0, (nc-1)), rowspan=1, colspan=1, anchor='W')

        self.fig.canvas.set_window_title(self.strTitle)
        self.fig.set_size_inches((settings.screen_cx * 0.33) / self.fig.dpi, (settings.screen_cy * 0.465) / self.fig.dpi)

        self.fig.canvas.mpl_connect('resize_event', self.onresize)
        self.fig.canvas.mpl_connect('close_event', self.onclose)

        self.fig.canvas.mpl_connect('button_press_event', self.onPress)
        self.fig.canvas.mpl_connect('button_release_event', self.onRelease)
        self.fig.canvas.mpl_connect('motion_notify_event', self.onMotion)

        self.scrollbar = scrollbar.VScrollbar(self.axSlider, callback=self.onScrollbarUpdate)

        plt.tight_layout()

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
        if (self.imgWidth == 0):
            return

        self.axesAspect = self.ax.figure.bbox_inches.width / self.ax.figure.bbox_inches.height
        self.axesHeight = self.imgWidth / self.axesAspect
        self.ax.set_xlim((0, self.imgWidth))
        self.ax.set_ylim((self.imgHeight, self.imgHeight - self.axesHeight))

        self.scrollbar.setScrollbarSize(self.imgHeight - self.axesHeight)
        self.scrollbar.setThumbSize(self.axesHeight)
        self.scrollbar.setPosition(0)

        if (self.selectedFrameMarker is not None):
            xx = self.ax.get_xlim()
            self.selectedFrameMarker.set_width(int(xx[1]-xx[0]))

    #------------------------------------------------------------------------------
    #
    def onPress(self, event):
        if event.inaxes != self.ax: return

        self.drag_active = True

        self.cur_xlim = self.ax.get_xlim()
        self.cur_ylim = self.ax.get_ylim()
        self.press = self.x0, self.y0, event.xdata, event.ydata
        self.x0, self.y0, self.xpress, self.ypress = self.press

        # set the child axes to pipe consequitive mouse events
        event.canvas.grab_mouse(self.ax)

    #------------------------------------------------------------------------------
    #
    def onMotion(self, event):
        if self.press is None: return
        if event.inaxes != self.ax: return
        if (self.drag_active == False): return

        #dx = event.xdata - self.xpress
        dy = event.ydata - self.ypress
        #self.cur_xlim -= dx
        self.cur_ylim -= dy

        if (self.cur_ylim[1] < 0.0):
            self.cur_ylim = (self.axesHeight, 0.0)
        elif (self.cur_ylim[0] > self.imgHeight):
            self.cur_ylim = (self.imgHeight, self.imgHeight - self.axesHeight)

        self.scrollbar.setPosition(self.imgHeight - self.cur_ylim[0])

        #self.ax.set_xlim(self.cur_xlim)
        self.ax.set_ylim(self.cur_ylim)
        self.fig.canvas.draw_idle()
        return

    #------------------------------------------------------------------------------
    #
    def onRelease(self, event):
        self.press = None
        self.drag_active = False
        # release the mouse grab held by the axes
        event.canvas.release_mouse(self.ax)

    #------------------------------------------------------------------------------
    #
    def onScrollbarUpdate(self, position):
        position = self.imgHeight - position
        self.ax.set_ylim((position, position - self.axesHeight))

    # -----------------------------------------------------------------------------
    #
    def updateInputName(self):
        self.fig.canvas.set_window_title(self.strTitle + ' - [' + settings.application.strBeautifiedInputFile + ']')

    #------------------------------------------------------------------------------
    #
    def saveView(self):
        if (self.fig is None):
            return

        filePath = os.path.join(settings.application.outputFolder, settings.application.outputName + '_LabanotationScore.png')
        filePath = settings.checkFileAlreadyExists(filePath, fileExt=".png", fileTypes=[('png files', '.png'), ('all files', '.*')])
        if (filePath is None):
            return

        try:
            self.fig.savefig(filePath, bbox_inches='tight')
            settings.application.logMessage("Labanotation score view was saved to '" + settings.beautifyPath(filePath) + "'")
        except Exception as e:
            strError = e
            settings.application.logMessage("Exception saving Labanotation score view to '" + settings.beautifyPath(filePath) + "': " + str(e))

    #------------------------------------------------------------------------------
    #
    def saveImage(self):
        if (self.view is None):
            return

        filePath = settings.application.outputFilePathImg
        filePath = settings.checkFileAlreadyExists(filePath, fileExt=".png", fileTypes=[('png files', '.png'), ('all files', '.*')])
        if (filePath is None):
            return

        try:
            cv2.imwrite(filePath, self.view.img)

            settings.application.logMessage("Labanotation score image was saved to '" + settings.beautifyPath(filePath) + "'")
        except Exception as e:
            strError = e
            settings.application.logMessage("Exception saving Labanotation score image to '" + settings.beautifyPath(filePath) + "': " + str(e))

    #------------------------------------------------------------------------------
    #
    def setLabanotation(self, timeS, all_laban):
        cnt = len(timeS)

        script = settings.application.labanotation.labanToScript(timeS, all_laban)

        s = 60
        self.view = settings.application.labanotation.labanScriptToImage(s * 10, s * cnt, script)

        if (self.im != None):
            self.ax.images.remove(self.im)

        if (self.selectedFrameMarker != None):
            self.ax.patches.remove(self.selectedFrameMarker)
            self.selectedFrameMarker = None

        self.ax.clear()
        self.im = self.ax.imshow(self.view.img, interpolation="bicubic", cmap=cm.gray)
        self.im.set_zorder(1)

        self.imgWidth = self.view.img.shape[1]-1
        self.imgHeight = self.view.img.shape[0]-1

        cx = self.imgWidth
        cy = self.imgHeight

        self.axesAspect = self.ax.figure.bbox_inches.width / self.ax.figure.bbox_inches.height
        self.axesHeight = self.imgWidth / self.axesAspect
        self.ax.set_xlim((0, self.imgWidth))
        self.ax.set_ylim((self.imgHeight, self.imgHeight - self.axesHeight))

        self.ax.get_xaxis().set_visible(False)
        self.ax.get_yaxis().set_visible(False)

        self.scrollbar.setScrollbarSize(self.imgHeight - self.axesHeight)
        self.scrollbar.setThumbSize(self.axesHeight)
        self.scrollbar.setPosition(0)

        self.selectTime(self.currentTime)

        self.fig.canvas.draw_idle()

    #------------------------------------------------------------------------------
    #
    def selectTime(self, time):
        if (self.view is None): return;

        self.currentTime = time

        padding = 3.0
        # time is [0..1]
        y = int(self.view.timeOffset) - int(time*self.view.timeScale)

        if (self.selectedFrameMarker is None):
            xx = self.ax.get_xlim()
            yy = self.ax.get_ylim()
            self.selectedFrameMarker = patches.Rectangle((xx[0], y-padding), (xx[1]-xx[0]), 2*padding, alpha=0.5, color='purple')
            self.selectedFrameMarker.set_zorder(10)
            self.ax.add_patch(self.selectedFrameMarker)
        else:
            self.selectedFrameMarker.set_y(y-padding)

        # scroll image into view
        halfHeight = (self.axesHeight / 2.0)
        position = y + halfHeight

        if (position < self.axesHeight):
            position = self.axesHeight
        if (position > self.imgHeight):
            position = self.imgHeight

        self.ax.set_ylim((position, position - self.axesHeight))

        # set scroll bar position
        self.scrollbar.setPosition(self.imgHeight - position)

        # update plot when idle
        self.fig.canvas.draw_idle()


