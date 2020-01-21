# --------------------------------------------------------------------------------------------
# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.
# --------------------------------------------------------------------------------------------

import os,sys

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.backend_tools import ToolBase, ToolToggleBase
from matplotlib.widgets import Slider, Cursor, Button, RadioButtons

import Tkinter
from Tkinter import *

import tkFileDialog
import tkMessageBox

# -----------------------------------------------------------------------------
#
def initialize():
    global appVersion           # application version
    global fVerbose             # verbose console output
    global application          # application object
    global cwd                  # current working directory
    global screen_cx            # screen width in pixels
    global screen_cy            # screen height in pixels
    global tkGuiCanvas          # holds _tkcanvas for guiMenu figure

    if (os.name == 'nt'):
        os.system('color')  # needed on windows platforms to support terminal colors

    appVersion = 'v1.00.0146'
    fVerbose = False
    application = None
    scale_canvas = 1.0

    cwd = os.getcwd()

    # If we're in the "src" directory, use the parent directory as the base path
    # for files to make specifying paths a little easier for the user
    if (cwd[-4:] == '\\src') or (cwd[-4:] == '/src'):
        cwd = cwd[0:-4]

    backend = matplotlib.get_backend()
    if (backend == 'TkAgg'):
        window = plt.get_current_fig_manager().window
        screen_cx, screen_cy = window.wm_maxsize()
    elif 'Qt' in backend:
        app = QtGui.QApplication(sys.argv)
        screen_resolution = app.desktop().screenGeometry()
        screen_cx, screen_cy = screen_resolution.width(), screen_resolution.height()

    # overwrite the screen size because on systems with multi-monitor configuration the larger 
    # screen resolution can create undesired window overlapping/shifting/clipping across monitors
    screen_cx, screen_cy = 1920, 1080

    plt.close('all')

    # initialize tk gui canvas to None. Once guiMenu is created, the variable
    # is reset
    tkGuiCanvas = None

    # alternative methods to get screen resolution
    if (False):
        # get the display resolution
        window = plt.get_current_fig_manager().window
        screen_cx, screen_cy = window.wm_maxsize()
        # or
        w = window.winfo_screenwidth()
        h = window.winfo_screenheight()
 
# -----------------------------------------------------------------------------
#
def beautifyPath(strPath):
    splitOutput = os.path.split(os.path.abspath(strPath))
    relativePath = application.getRelativePath(cwd, splitOutput[0])
    return os.path.join(relativePath, splitOutput[1])

# -----------------------------------------------------------------------------
#
def OLD_getTextExtent(ax, str='Testy', fontsize=10, fontweight='normal'):
    renderer = ax.figure.canvas.get_renderer()
    t = ax.text(0.0, 0.0, str, transform=ax.transData, color='r', fontsize=fontsize, fontweight=fontweight)
    ax.figure.canvas.draw()

    (w, h, d) = renderer.get_text_width_height_descent(str, t._fontproperties, False)
    (x1, y1) = ax.transData.inverted().transform((0, 0))
    (x2, y2) = ax.transData.inverted().transform((w, h+d))

    t.remove()

    return ((x2-x1), (y2-y1))

# -----------------------------------------------------------------------------
#
def getTextExtent(ax, str='Testy', fontsize=None, fontweight='normal'):
    renderer = ax.figure.canvas.get_renderer()
    t = ax.text(0.0, 0.0, str, transform=ax.transData, color='r', fontsize=fontsize, fontweight=fontweight, bbox={'facecolor':'lightgray', 'alpha':0.7, 'pad':0.0})
    ax.figure.canvas.draw()

    (w, h, d) = renderer.get_text_width_height_descent(str, t._fontproperties, False)
    (x1, y1) = ax.transData.inverted().transform((0, 0))
    (x2, y2) = ax.transData.inverted().transform((w, h+d))

    t.remove()

    return ((x2-x1), (y2-y1))

# -----------------------------------------------------------------------------
# get figure's position in pixels (x, y, dx, dy)
#
def getCanvasPosition(fig):
    mgr = plt.get_current_fig_manager()
    backend = matplotlib.get_backend()
    if (backend == 'TkAgg'):
        # x, y, width, height = fig.canvas.manager.window.get_geometry()
           
        str = fig.canvas.manager.window.wm_geometry()
        split = str.split('+')
        if (len(split) == 2):
            return (int(split[0]), int(split[1]))
        elif (len(split) == 3):
            return (int(split[1]), int(split[2]))

        return (0, 0)

    elif 'WX' in backend:
        p = mgr.window.GetPosition()
        # s = mgr.window.GetSize()
        return (p[0], p[1])

    elif 'QT' in backend:
        CurPos = mgr.window.geometry().getRect()
        return (CurPos[0], CurPos[2])
    else:
        return (0, 0)
    
    # This works for QT and GTK. Can also use window.setGeometry
    geom = fig.canvas.manager.window.geometry()
    x,y,dx,dy = geom.getRect()
    return (x,y)

# -----------------------------------------------------------------------------
# move figure's upper left window corner to pixel (x, y)
#
def moveCanvas(fig, x, y):
    backend = matplotlib.get_backend()
    if (backend == 'TkAgg'):
        fig.canvas.manager.window.wm_geometry("+%d+%d" % (x, y))
    elif (backend == 'WXAgg'):
        fig.canvas.manager.window.SetPosition((x, y))
    else:
        # This works for QT and GTK. Can also use window.setGeometry
        fig.canvas.manager.window.move(x, y)

# -----------------------------------------------------------------------------
# center figure canvas on screen
#
def centerFigure(win):
    win.update_idletasks()
    width = win.winfo_width()
    height = win.winfo_height()
    x = (win.winfo_screenwidth() // 2) - (width // 2)
    y = (win.winfo_screenheight() // 2) - (height // 2)
    win.geometry('{}x{}+{}+{}'.format(width, height, x, y))

# -----------------------------------------------------------------------------
# check whether file already exists, and present action dialog if necessary
#
def checkFileAlreadyExists(filePath, fileExt=".txt", fileTypes=[('text files', '.txt'), ('all files', '.*')]):
    options = {}

    if (tkGuiCanvas is not None):
        options['parent'] = tkGuiCanvas

    if (not os.path.isfile(filePath)):
        return filePath

    result = tkMessageBox.askyesnocancel("Labanotation", "The file '" + filePath + "' already exists.\r\n\r\n\tChoose 'Yes' to overwrite.\r\n\tChoose 'No' to choose a new file name.\r\n\tChoose 'Cancel' to cancel.\r\n", **options)
    if (result is True):
        return filePath
    elif (result is None):
        return None

    # define options for dialog
    splitInput = os.path.split(os.path.abspath(filePath))
    if  splitInput[1] != '':
        fileName = splitInput[1]
    else:
        fileName = splitInput[0]

    options = {}
    options['defaultextension'] = fileExt
    options['filetypes'] = fileTypes
    options['initialdir'] = os.path.dirname(filePath)
    options['initialfile'] = fileName
    options['title'] = "Labanotation"

    if (tkGuiCanvas is not None):
        options['parent'] = tkGuiCanvas

    response = tkFileDialog.asksaveasfilename(**options)
    if (response is ''):
        return None

    return response
