# --------------------------------------------------------------------------------------------
# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.
# --------------------------------------------------------------------------------------------

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.backend_tools import ToolBase, ToolToggleBase
from matplotlib.widgets import AxesWidget

import matplotlib.patches as patches

# -----------------------------------------------------------------------------
#
class VScrollbar(AxesWidget):
    # -----------------------------------------------------------------------------
    #
    def __init__(self, ax, scrollbarSize=1.0, thumbSize=0.3, position=0.0, callback=None):
        AxesWidget.__init__(self, ax)

        self.ax = ax
        self.ax.get_xaxis().set_visible(False)
        self.ax.get_yaxis().set_visible(False)
        self.ax.set_ylim((0, scrollbarSize + thumbSize))

        self.scrollbarSize = scrollbarSize
        self.scrollPage = (self.scrollbarSize / 20)
        self.thumbSize = thumbSize
        self.position = position

        self.callback = callback

        xx = self.ax.get_xlim()
        width = xx[1]-xx[0]
        height = self.thumbSize

        self.thumb = patches.Rectangle((xx[0], self.position), width, height, alpha=1.0, color='gray')
        self.ax.add_patch(self.thumb)
        self.bbox = self.thumb.get_bbox()

        self.drag_active = False
        self.drag_startY = 0

        self.ax.edgecolor = 'blue'
        self.ax.set_facecolor('0.90')

        self.connect_event('button_press_event', self.onPress)
        self.connect_event('button_release_event', self.onRelease)
        self.connect_event('motion_notify_event', self.onMotion)

    #------------------------------------------------------------------------------
    #
    def onPress(self, event):
        if self.ignore(event):
            return

        if event.inaxes != self.ax: return

        contains = self.thumb.contains(event)[0]
        if (contains):
            self.drag_active = True
            self.drag_startY = event.ydata
            self.drag_offset = (event.ydata - self.position)
        elif (event.ydata < self.position):
            self.setPosition(self.position - self.scrollPage, True)
        else:
            self.setPosition(self.position + self.scrollPage, True)

        # set the child axes to pipe consequitive mouse events
        event.canvas.grab_mouse(self.ax)

    #------------------------------------------------------------------------------
    #
    def onMotion(self, event):
        if self.ignore(event):
            return

        if (event.ydata is None):
            return

        if (self.drag_active):
            diff = (event.ydata - self.drag_startY)
            newY = event.ydata - self.drag_offset
            self.setPosition(newY, True)

    #------------------------------------------------------------------------------
    #
    def onRelease(self, event):
        if self.ignore(event):
            return

        self.drag_active = False
        # release the mouse grab held by the axes
        event.canvas.release_mouse(self.ax)

    #------------------------------------------------------------------------------
    #
    def setScrollbarSize(self, scrollbarSize):
        self.scrollbarSize = scrollbarSize
        self.scrollPage = (self.scrollbarSize / 20)

        self.ax.set_ylim((0, (self.scrollbarSize + self.thumbSize)))

        self.update()

    #------------------------------------------------------------------------------
    #
    def setThumbSize(self, thumbSize):
        self.thumbSize = thumbSize

        self.ax.set_ylim((0, (self.scrollbarSize + self.thumbSize)))

        self.update()

    #------------------------------------------------------------------------------
    #
    def setPosition(self, position, notifyCallback=False):
        if (position < 0):
            position = 0
        elif (position > self.scrollbarSize):
            position = self.scrollbarSize

        self.position = position
        self.update()

        if ((self.callback is not None) and (notifyCallback)):
            self.callback(self.position)

    #------------------------------------------------------------------------------
    #
    def update(self):
        if (self.thumb is None):
            return

        self.thumb.set_y(self.position)
        self.thumb.set_height(self.thumbSize)
        self.bbox = self.thumb.get_bbox()
        self.ax.figure.canvas.draw_idle()

