# --------------------------------------------------------------------------------------------
# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.
# --------------------------------------------------------------------------------------------

import settings

from . import algnaive
from . import algtotal
from . import algparallel
from . import labanProcessor
from . import labanVisualization

class labanotation:
    all_laban = []
    timeS = None
    algorithm = None
    idEventC = None
    idEventR = None
    idEventM = None

    #------------------------------------------------------------------------------
    # Class initialization
    #
    def __init__(self):
        print ('Initializing Labanotation...')

    #------------------------------------------------------------------------------
    # convert joint data frames to labanotation using specified algorithm
    #
    def ensureAlgorithmObject(self, algorithm):
        algorithm = algorithm.lower()
        if (algorithm == 'naive'):
            if ((self.algorithm == None) or (self.algorithm.algorithm != algorithm)):
                self.algorithm = algnaive.Algorithm(algorithm)
                self.setupButtonEvents();
        elif (algorithm == 'total'):
            if ((self.algorithm == None) or (self.algorithm.algorithm != algorithm)):
                self.algorithm = algtotal.Algorithm(algorithm)
                self.setupButtonEvents();
        elif (algorithm == 'parallel'):
            if ((self.algorithm == None) or (self.algorithm.algorithm != algorithm)):
                self.algorithm = algparallel.Algorithm(algorithm)
                self.setupButtonEvents();
        else:
            self.algorithm = None
            print("Internal Error: Unkown alogirthm '" + algorithm + "'.")

    #------------------------------------------------------------------------------
    # convert joint data frames to labanotation using specified algorithm
    #
    def applyAlgorithm(self, ax, jointD, algorithm, forceReset = False):
        self.ensureAlgorithmObject(algorithm)

        if (self.algorithm == None):
            print("Internal Error: Unkown alogirthm '" + algorithm + "'.")
            return (None, None)

        return self.algorithm.convertToLabanotation(ax, jointD, forceReset)

    #------------------------------------------------------------------------------
    #
    def getGaussianParameters(self, algorithm):
        algorithm = algorithm.lower()
        gauss_params = (31, 5)
        self.ensureAlgorithmObject(algorithm)
        if (self.algorithm is None):
            print("Internal Error: Unkown alogirthm '" + algorithm + "'.")
            return gauss_params

        if (algorithm == 'naive'):
            pass
        elif (algorithm == 'total'):
            gauss_params = (self.algorithm.gauss_window_size, self.algorithm.gauss_sigma)
        elif (algorithm == 'parallel'):
            gauss_params = (self.algorithm.gauss_window_size, self.algorithm.gauss_sigma)
        else:
            print("Internal Error: Unkown alogirthm '" + algorithm + "'.")

        return gauss_params

    #------------------------------------------------------------------------------
    #
    def setGaussianParameters(self, algorithm, gauss_params):
        algorithm = algorithm.lower()

        self.ensureAlgorithmObject(algorithm)
        if (self.algorithm is None):
            return

        if (algorithm == 'naive'):
            pass
        elif (algorithm == 'total'):
            self.algorithm.gauss_window_size = gauss_params[0]
            self.algorithm.gauss_sigma = gauss_params[1]
        elif (algorithm == 'parallel'):
            self.algorithm.gauss_window_size = gauss_params[0]
            self.algorithm.gauss_sigma = gauss_params[1]
        else:
            print("Internal Error: Unkown alogirthm '" + algorithm + "'.")

    #------------------------------------------------------------------------------
    #
    def setupButtonEvents(self):
        if (settings.application.graphFilter is not None):
            canvas = settings.application.graphFilter.fig.canvas

            if (self.idEventC is not None):
                canvas.mpl_disconnect(self.idEventC)
                self.idEventC = None

            if (self.idEventR is not None):
                canvas.mpl_disconnect(self.idEventR)
                self.idEventR = None

            if (self.idEventM is not None):
                canvas.mpl_disconnect(self.idEventM)
                self.idEventM = None

            if (self.algorithm != None):
                self.idEventC = canvas.mpl_connect('button_press_event', self.algorithm.onCanvasClick)
                self.idEventR = canvas.mpl_connect('button_release_event', self.algorithm.onCanvasRelease)
                self.idEventM = canvas.mpl_connect('motion_notify_event', self.algorithm.onCanvasMove)

    #------------------------------------------------------------------------------
    #
    def labanToScript(self, timeS, all_laban):
        return labanProcessor.toScript(timeS, all_laban)

    #------------------------------------------------------------------------------
    #
    def labanScriptToImage(self, w, h, script):
        return labanVisualization.convertLabanScriptToView(w, h, script);

    #------------------------------------------------------------------------------
    #
    def saveToJSON(self):
        if (self.algorithm != None):
            self.algorithm.saveToJSON()

    #------------------------------------------------------------------------------
    #
    def saveToTXT(self):
        if (self.algorithm != None):
            self.algorithm.saveToTXT()

    #------------------------------------------------------------------------------
    #
    def selectTime(self, time):
        if (self.algorithm != None):
            self.algorithm.selectTime(time)
