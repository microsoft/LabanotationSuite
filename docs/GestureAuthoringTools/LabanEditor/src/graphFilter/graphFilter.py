# --------------------------------------------------------------------------------------------
# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.
# --------------------------------------------------------------------------------------------

import os, math, copy
import numpy as np
from decimal import Decimal
from collections import OrderedDict

import matplotlib.pyplot as plt
plt.rcParams['toolbar'] = 'None'
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backend_tools import ToolBase, ToolToggleBase
from matplotlib.widgets import Slider, Cursor, Button, RadioButtons
from mpl_toolkits.axes_grid1.inset_locator import InsetPosition
import matplotlib.patches as patches

import tkMessageBox

import settings

class graphFilter:
    fig = None
    ax = None

    #------------------------------------------------------------------------------
    # Class initialization
    #
    def __init__(self):
        self.strTitle = 'Filter Graph/Key Frame Editor'
        self.fig, self.ax = plt.subplots()
        self.fig.canvas.set_window_title(self.strTitle)
        self.fig.set_size_inches((settings.screen_cx * 0.65) / self.fig.dpi, (settings.screen_cy * 0.465) / self.fig.dpi)

        self.fig.canvas.mpl_connect('resize_event', self.onresize)
        self.fig.canvas.mpl_connect('close_event', self.onclose)

        self.cursor = Cursor(self.ax, useblit=True, color='red', linewidth=0.5)

        # default is (0, 0, 1, 1) - [left, bottom, right, top]
        plt.tight_layout(rect=[0.02, 0.03, 0.97, 0.95])

        # create info help button
        strBtnText = 'i'
        btnFontSize = 14
        cx, cy = settings.getTextExtent(self.ax, strBtnText, fontsize=btnFontSize, fontweight='bold')
        cx = cx * 4.0
        cy = cy * 1.8

        self.button_ax = plt.axes([0, 0, 1, 1])
        ip = InsetPosition(self.ax, [1.0-cx, 1.0 + (cy/4), cx, cy]) #posx, posy, width, height
        self.button_ax.set_axes_locator(ip)
        self.btnInfo = Button(self.button_ax, strBtnText, color='blue', hovercolor='lightblue')
        self.btnInfo.label.set_color('w')
        self.btnInfo.label.set_fontsize(btnFontSize)
        self.btnInfo.label.set_fontweight('bold')
        self.btnInfo.on_clicked(self.onclickInfoButton)

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
        pass

    # -----------------------------------------------------------------------------
    #
    def onclickInfoButton(self, event):
        tk = None
        try:
            if (self.fig.canvas is not None) and (self.fig.canvas._tkcanvas is not None):
                tk = self.fig.canvas._tkcanvas
        except Exception as e:
            pass

        tkMessageBox.showinfo("Labanotation",
            "Key Frame (green star icon) Editing Instructions:\r\n\r\n\r\n"
            "  - Right-click and drag on key frame star to *MOVE* along graph line.\r\n\r\n"
            "  - Right-click on graph line to *CREATE* a new key frame star.\r\n\r\n"
            "  - Double right-click to *DELETE* key frame star.\r\n\r\n", parent=tk)

    # -----------------------------------------------------------------------------
    #
    def updateInputName(self):
        self.fig.canvas.set_window_title(self.strTitle + ' - [' + settings.application.strBeautifiedInputFile + ']')

    #------------------------------------------------------------------------------
    #
    def saveView(self):
        if (self.fig is None):
            return

        filePath = os.path.join(settings.application.outputFolder, settings.application.outputName + '_FilterGraph.png')
        filePath = settings.checkFileAlreadyExists(filePath, fileExt=".png", fileTypes=[('png files', '.png'), ('all files', '.*')])
        if (filePath is None):
            return

        # hide info button so it won't appear in image. Force an immediate redraw
        self.button_ax.set_visible(False)
        self.fig.canvas.draw()

        try:
            self.fig.savefig(filePath, bbox_inches='tight')
            settings.application.logMessage("Filter Graph view was saved to '" + settings.beautifyPath(filePath) + "'")
        except Exception as e:
            strError = e
            settings.application.logMessage("Exception saving Filter Graph view to '" + settings.beautifyPath(filePath) + "': " + str(e))


        # show info button.
        self.button_ax.set_visible(True)
        self.fig.canvas.draw_idle()

    #------------------------------------------------------------------------------
    #
    def selectTime(self, time):
        if (settings.application.labanotation != None):
            settings.application.labanotation.selectTime(time)
            self.fig.canvas.draw_idle()

    #------------------------------------------------------------------------------
    #
