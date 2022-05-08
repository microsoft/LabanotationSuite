# --------------------------------------------------------------------------------------------
# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.
# --------------------------------------------------------------------------------------------

import os,sys,inspect,errno
import argparse
import numpy as np

# import matplotlib
# matplotlib.use('Qt5Agg')

import matplotlib.pyplot as plt

import tkMessageBox

import settings

import kinect as kinect

import guiMenu.guiMenu as guiMenu
import graphSkeleton.graphSkeleton as graphSkeleton
import graphFilter.graphFilter as graphFilter
import graphLaban.graphLaban as graphLaban
import labanotation.labanotation as labanotation

# -----------------------------------------------------------------------------
#
class application:
    # flags
    algorithm = None
    inputFilePath = None
    inputName = None
    outputFilePathTxt = None
    outputFilePathJson = None
    outputFilePathImg = None
    outputName = None
    fShowGUI = True

    # objects
    guiMenu = None
    graphSkeleton = None
    graphFilter = None
    graphLaban = None
    labanotation = None

    # data
    jointFrames = []
    all_laban = None
    timeS = None

    #------------------------------------------------------------------------------
    # Class initialization
    #
    def __init__(self):
        # set global application variable now so all other objects have access 
        # to the application object.
        settings.application = self

        # print('\033[4m\033[1m' + 'Labanotation ' + settings.appVersion + '\033[0m')
        print('Labanotation ' + settings.appVersion + '\r\n')

        self.parseArguments()

        if (self.fShowGUI):
            # create a graph for the 3d skeletons
            self.graphSkeleton = graphSkeleton.graph3D()

            # create a figure for the filter graph
            self.graphFilter = graphFilter.graphFilter()

            # create a graph for the labanotation visualization
            self.graphLaban = graphLaban.graphLaban()

            # create a figure for the UI
            self.guiMenu = guiMenu.guiMenu()

            self.organizeCanvas()

        # create labanotation object
        self.labanotation = labanotation.labanotation()

        # now that all objects have been created, reset gaussian parameters 
        # in guiMenu to reflect selected algorithm's gaussian values
        if (self.guiMenu is not None):
            self.guiMenu.gauss_params = settings.application.labanotation.getGaussianParameters(self.guiMenu.getAlgorithmSelection())
            self.guiMenu.updateUIControls(self.algorithm)

    #------------------------------------------------------------------------------
    # parse command line arguments
    #
    def parseArguments(self):
        parser = argparse.ArgumentParser(description='Kinect .csv to Labanotation .json gesture keyframe extractor.')

        parser.add_argument('--algorithm', default='total', choices=['total', 'parallel', 'naive'], help='select the Laban key frame extraction algorithm: "total" total energy, "parallel" parallel energy, "naive" treat each Kinect frame as a keyframe')
        parser.add_argument('--base-rotation-style', default='every',
                            choices=['every', 'first'], help='select the base rotation style. "every": update base rotation every frame. "first": use the base rotation of the first frame.')
        parser.add_argument('--inputfile', help='Kinect data input file')
        parser.add_argument('--nogui', action='store_true', default=False, help='process Kinect data but don\'t display interactive GUI')
        parser.add_argument('--outputfolder', help='output folder; if not specified, output folder is .\\data_output')
        parser.add_argument('--screenwidth', default=1920, help='overwrite default screen width')
        parser.add_argument('--screenheight', default=1080, help='overwrite default screen height')

        cmdArgs = parser.parse_args()

        if (cmdArgs.inputfile == None):
            print('No inputfile was specified.')
            exit()
        elif (cmdArgs.inputfile != None) and (cmdArgs.inputfile != ''):
            self.inputName = cmdArgs.inputfile
    
        self.algorithm = cmdArgs.algorithm
        self.outputFolder = cmdArgs.outputfolder
        self.fShowGUI = not cmdArgs.nogui
        self.determineFilePaths()
        self.base_rotation_style = cmdArgs.base_rotation_style

        # overwrite default screen settings
        settings.screen_cx = int(cmdArgs.screenwidth)
        settings.screen_cy = int(cmdArgs.screenheight)

    # -----------------------------------------------------------------------------
    # determine input and various output file paths
    #
    def determineFilePaths(self):
        # determine absolute input file path
        if os.path.isabs(self.inputName):
            self.inputFilePath = self.inputName
        else:
            self.inputFilePath = os.path.join(settings.cwd, 'data_input', self.inputName)

        # make inputName file name only
        splitInput = os.path.split(os.path.abspath(self.inputFilePath))
        if  splitInput[1] != '':
            self.inputName = splitInput[1]

        if os.path.splitext(self.inputName)[1] == '':
            self.inputFilePath = self.inputFilePath + '.csv'

        # remove file extension from inputName, if any
        inputNameSplit = os.path.splitext(self.inputName)
        if  inputNameSplit[1] != '':
            self.inputName = inputNameSplit[0]

        # determine output file paths
        self.outputName = self.inputName

        if (self.outputFolder is None):
            self.outputFolder = os.path.join(settings.cwd, 'data_output')

        # make sure output folder exists.
        if not os.path.exists(self.outputFolder):
            try:
                os.makedirs(self.outputFolder)
            except OSError as e:
                if e.errno != errno.EEXIST:
                    raise

        self.outputFilePathTxt = os.path.join(self.outputFolder, self.outputName + '.txt')
        self.outputFilePathJson = os.path.join(self.outputFolder, self.outputName + '.json')
        self.outputFilePathImg = os.path.join(self.outputFolder, self.outputName + '.png')

        # make beautified input and output file path
        splitInput = os.path.split(os.path.abspath(self.inputFilePath))
        relativePath = self.getRelativePath(settings.cwd, splitInput[0])
        self.strBeautifiedInputFile = os.path.join(relativePath, splitInput[1])

        splitOutput = os.path.split(os.path.abspath(self.outputFilePathJson))
        relativePath = self.getRelativePath(settings.cwd, splitOutput[0])
        self.strBeautifiedOutputFile = os.path.join(relativePath, splitOutput[1])

    # -----------------------------------------------------------------------------
    #
    def logMessage(self, text, ioRedirect=False):
        # print(text)
        if (self.guiMenu is not None):
            self.guiMenu.logMessage(text, ioRedirect)

    # -----------------------------------------------------------------------------
    # layout and position all the figures across the screen
    #
    def organizeCanvas(self):
        settings.moveCanvas(self.guiMenu.fig, 10, 10)

        xMenu, yMenu = settings.getCanvasPosition(self.guiMenu.fig)
        szMenu = self.guiMenu.fig.get_size_inches() * self.guiMenu.fig.dpi

        settings.moveCanvas(self.graphSkeleton.fig, xMenu + szMenu[0] + 20, yMenu)
        settings.moveCanvas(self.graphFilter.fig, xMenu, yMenu + szMenu[1] + 40)

        szS = self.graphSkeleton.fig.get_size_inches() * self.graphSkeleton.fig.dpi
        szF = self.graphFilter.fig.get_size_inches() * self.graphFilter.fig.dpi
        settings.moveCanvas(self.graphLaban.fig, xMenu + szF[0] + 20, yMenu + szS[1] + 40)

    # -----------------------------------------------------------------------------
    # close()  - Called by class objects that have a canvas and the user closed 
    # that canvas/figure
    #
    def close(self):
        # When one window is closed, close all of them
        plt.close('all')

    # -----------------------------------------------------------------------------
    # when possible, create a relative sub_path for path from main_path
    #
    def getRelativePath(self, main_path, sub_path):
        if sys.platform == "win32":
            main_path = main_path.lower()
            sub_path = sub_path.lower()
        _main_path = os.path.abspath(main_path).split(os.path.sep)
        _sub_path = os.path.abspath(sub_path).split(os.path.sep)
        eq_until_pos = None
        for i in xrange(min(len(_main_path), len(_sub_path))):
            if _main_path[i] == _sub_path[i]:
                eq_until_pos = i
            else:
                break
        if eq_until_pos is None:
            return sub_path
        newpath = [".." for i in xrange(len(_main_path[eq_until_pos+1:]))]
        newpath.extend(_sub_path[eq_until_pos+1:])
        return os.path.join(*newpath) if newpath else "."

    #------------------------------------------------------------------------------
    #
    def loadKinectDataFile(self):
        self.logMessage('Reading input data file ' + self.strBeautifiedInputFile)

        self.jointFrames = kinect.loadKinectDataFile(self.inputFilePath, True)

        self.logMessage('Loaded ' + str(len(self.jointFrames)) + ' Kinect data frames.')
        if (len(self.jointFrames) < 1):
            self.logMessage('No Kinect data frames to process.')
            return

        if (self.graphSkeleton is not None):
            self.graphSkeleton.setJointFrames(self.jointFrames)

    #------------------------------------------------------------------------------
    #
    def selectTime(self, time, caller=None):
        if (self.graphSkeleton is not None):
            if (caller is not self.graphSkeleton):
                self.graphSkeleton.selectTime(time, True)
        else:
            print('Internal Error: No skeleton graph.')

        if (self.graphFilter is not None):
            # always select time for graphFilter
            self.graphFilter.selectTime(time)
        else:
            print('Internal Error: No filter graph.')

        if (self.graphLaban is not None):
            self.graphLaban.selectTime(time)

    #------------------------------------------------------------------------------
    #
    def applyAlgoritm(self, algorithm, forceReset = False):
        self.algorithm = algorithm.lower()

        self.logMessage("Applying algorihm '" + self.algorithm + "'...")

        ax = self.graphFilter.ax if (self.graphFilter != None) else None

        [self.timeS, self.all_laban] = self.labanotation.applyAlgorithm(ax, self.jointFrames, self.algorithm, forceReset,
                                                                        base_rotation_style=self.base_rotation_style)

        # share labanotation with skeleton graph and laban visualizer
        if (self.graphSkeleton != None):
            self.graphSkeleton.setLabanotation(self.timeS, self.all_laban)

        if (self.graphLaban != None):
            self.graphLaban.setLabanotation(self.timeS, self.all_laban)

        if (self.graphFilter != None):
            self.graphFilter.fig.canvas.draw_idle()

    #------------------------------------------------------------------------------
    #
    def updateLaban(self, timeS, all_laban):
        self.timeS = timeS
        self.all_laban = all_laban

        if (self.graphSkeleton != None):
            self.graphSkeleton.setLabanotation(self.timeS, self.all_laban)

        if (self.graphLaban != None):
            self.graphLaban.setLabanotation(self.timeS, self.all_laban)

    #------------------------------------------------------------------------------
    #
    def saveJSON(self):
        if (self.labanotation != None):
            try:
                self.labanotation.saveToJSON()
            except Exception as e:
                strError = e
                self.logMessage('Exception saving to JSON file: ' + str(e))

    #------------------------------------------------------------------------------
    #
    def saveTXT(self):
        if (self.labanotation != None):
            try:
                self.labanotation.saveToTXT()
            except Exception as e:
                strError = e
                self.logMessage('Exception saving to text file: ' + str(e))

    #------------------------------------------------------------------------------
    #
    def saveImage(self):
        if (self.graphLaban != None):
            try:
                self.graphLaban.saveImage()
            except Exception as e:
                strError = e
                self.logMessage('Exception saving to image file: ' + str(e))

    #------------------------------------------------------------------------------
    #
    def saveFilterView(self):
        if (self.graphFilter != None):
            self.graphFilter.saveView()

    #------------------------------------------------------------------------------
    #
    def saveSkeletonView(self):
        if (self.graphSkeleton != None):
            self.graphSkeleton.saveView()

    #------------------------------------------------------------------------------
    #
    def saveScoreView(self):
        if (self.graphLaban != None):
            self.graphLaban.saveView()

    #------------------------------------------------------------------------------
    #
    def openAndProcessInputfile(self, file = None):
        forceReset = False

        if (file is not None):
            if (os.path.isabs(file)):
                self.inputName = file
                self.outputName = None
                self.determineFilePaths()

                # this is a new file. Force an algorithm reset
                forceReset = True
            else:
                self.logMessage("Error: The file '" + str(file) + "' is not a valid file.")
                tkMessageBox.showerror("Labanotation", "The file '" + str(file) + "' is not a valid file.", parent=settings.tkGuiCanvas)
                return

        # load Kinect capture data
        self.loadKinectDataFile()

        if (self.guiMenu != None):
            self.guiMenu.updateInputName()

        if (self.graphSkeleton != None):
            self.graphSkeleton.updateInputName()

        if (self.graphFilter != None):
            self.graphFilter.updateInputName()

        if (self.graphLaban != None):
            self.graphLaban.updateInputName()

        # apply selected algorithm
        self.applyAlgoritm(self.algorithm, forceReset)

        if (self.fShowGUI):
            # render skeleton(s) at time 0
            self.selectTime(0)
        else:
            # write to file
            self.saveJSON()

    #------------------------------------------------------------------------------
    #
    def run(self):
        self.openAndProcessInputfile()

        if (self.fShowGUI):
            # show all the plots and wait for user input
            plt.show()

# -----------------------------------------------------------------------------
# applet's main entry point
#
if __name__ == '__main__':
    # initialize global variables
    settings.initialize()

    # create main application object, then run it
    settings.application = application()
    settings.application.run()

    exit()
