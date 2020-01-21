#!/usr/bin/env python

# @package docstring
# Provide explanation of this code here.
# This code should be independent from ROS.
#
# More Details.
# Disclaimer and liscensing.
# Author information and email address to contact.
# Date of release.

import sys
import cv2
import numpy as np
import labanotationpy


# Class to draw labanotation score from a labanotation class object.
class LabanotationDrawer:

    # The constructor.
    def __init__(self):

        # Labanotation score parameters

        # Simple labanotation score has 11 spaces
        self._numberOfStaffSpaces = 11

        # Canvas (output image) parameters

        # The image size is not scalable to different values as of now
        self._width = 600
        self._height = 900  # Height of score excluding 0th frame
        self._pageBottomMargin = 160  # We draw our 0th frame here
        self._img = np.full((self._height + self._pageBottomMargin,
                             self._width),
                            255, np.uint8)
        self._pixelsPerStaffSpaces \
            = int(self._width // self._numberOfStaffSpaces)

    # Function to initialize canvas (draw staff for the score)
    #  @param self            The object pointer.
    #  @param pixelsPerSecond Parameter dependent on the loaded labanotation.
    def initCanvas(self, pixelsPerSecond):

        # clean canvas
        self._img = np.full((self._height + self._pageBottomMargin,
                             self._width),
                            255, np.uint8)

        # Draw vertical solid lines
        # Staff line in the center of the score
        cv2.line(self._img,
                 (self._pixelsPerStaffSpaces*5, 0),
                 (self._pixelsPerStaffSpaces*5, self._height),
                 0, 2)
        # Staff line between left leg and left arm
        cv2.line(self._img,
                 (self._pixelsPerStaffSpaces*3, 0),
                 (self._pixelsPerStaffSpaces*3, self._height),
                 0, 2)
        # Staff line between right leg and left leg
        cv2.line(self._img,
                 (self._pixelsPerStaffSpaces*7, 0),
                 (self._pixelsPerStaffSpaces*7, self._height),
                 0, 2)

        # Draw vertical dashed lines for other score lines
        for i in range(1, self._numberOfStaffSpaces):
            dashLength = 20
            x = self._pixelsPerStaffSpaces * i
            # dashLength << 1 = dash line length + space between next dash line
            for j in range(self._height // (dashLength << 1)):
                y = self._height - j*(dashLength << 1)
                cv2.line(self._img, (x, y), (x, y-dashLength), 0, 2)
            # Make sure that the line does not end w/ a white space at top
            edgeOfVeryTopDash \
                = (self._height // (dashLength << 1)) * (dashLength << 1)
            if self._height - edgeOfVeryTopDash > 0:
                cv2.line(self._img,
                         (x, self._height-edgeOfVeryTopDash), (x, 0),
                         0, 2)

        # Draw horizontal lines for the leg staffs (double line)
        cv2.line(self._img,
                 (self._pixelsPerStaffSpaces*3, self._height),
                 (self._pixelsPerStaffSpaces*7, self._height),
                 0, 2)
        cv2.line(self._img,
                 (self._pixelsPerStaffSpaces*3, self._height+4),
                 (self._pixelsPerStaffSpaces*7, self._height+4),
                 0, 2)

        # Draw small horizontal lines on the center staff line to indicate time
        y = self._height
        while y >= 0:
            cv2.line(self._img,
                     (self._pixelsPerStaffSpaces*5-3, y),
                     (self._pixelsPerStaffSpaces*5+3, y),
                     0, 2)
            y -= pixelsPerSecond

        # Write titles and subtitles
        cv2.putText(self._img, 'lower',
                    (self._pixelsPerStaffSpaces*0+5,
                     self._height+self._pageBottomMargin-50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1, 2)
        cv2.putText(self._img, 'upper',
                    (self._pixelsPerStaffSpaces*1+5,
                     self._height+self._pageBottomMargin-50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1, 2)
        cv2.putText(self._img, 'upper',
                    (self._pixelsPerStaffSpaces*8+5,
                     self._height+self._pageBottomMargin-50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1, 2)
        cv2.putText(self._img, 'lower',
                    (self._pixelsPerStaffSpaces*9+5,
                     self._height+self._pageBottomMargin-50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1, 2)
        cv2.putText(self._img, 'head',
                    (self._pixelsPerStaffSpaces*10-5,
                     self._height+self._pageBottomMargin-20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, 1, 2)
        cv2.putText(self._img, 'arm(L)',
                    (self._pixelsPerStaffSpaces*0+10,
                     self._height+self._pageBottomMargin-20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, 1, 2)
        cv2.putText(self._img, 'arm(R)',
                    (self._pixelsPerStaffSpaces*8+10,
                     self._height+self._pageBottomMargin-20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, 1, 2)

    # Function to draw the background pattern of a labanotation symbol.
    # Background pattern depends on the labanotation level.
    #  @param self     The object pointer.
    #  @param levels   Array of labanotation levels of a frame.
    #  @param drawFlag Body part array indicating whether to draw the part.
    #  @param xs       Body part array indicating the draw area x-values.
    #  @param yTop     Top y-value of the canvas draw area.
    #  @param yBottom  Bottom y-value of the canvas draw area.
    def drawSymbolBackgroundPattern(self, levels, drawFlag, xs, yTop, yBottom):
        for bp in range(len(levels)):
            if not drawFlag[bp]:
                continue
            if levels[bp] == labanotationpy.MIDDLE:
                # Symbol draws a circle in the middle
                cv2.circle(self._img,
                           ((xs[bp][0]+xs[bp][1])//2, (yTop+yBottom)//2),
                           4, 0, -1)
            elif levels[bp] == labanotationpy.HIGH:
                # Symbol draws shading lines
                shadingLineIntervals = 20
                # End points of 0th shading line are placed on the upper-left
                xShadingLineLeftEnd = xs[bp][0]
                yShadingLineLeftEnd = yTop
                xShadingLineRightEnd = xs[bp][0]
                yShadingLineRightEnd = yTop
                # Keep moving the left end along the y-axis
                # Keep moving the right end along the x-axis
                # Both end points are moved at the same time
                # A line is created everytime the end points are moved
                # Once the left end reaches the bottom-left,
                # it will move along the x-axis
                # Once the right end reaches the upper-right,
                # it will move along the y-axis
                # Break condition is one of below:
                #   1. When the left end reaches the bottom-right corner
                #   2. When the right end reaches the bottom-right corner
                while True:
                    # Move the end points
                    yShadingLineLeftEnd += shadingLineIntervals
                    xShadingLineRightEnd += shadingLineIntervals
                    # As default, left end is still on the y-axis,
                    #             right end on the x-axis
                    xRemappedLeftEnd = xShadingLineLeftEnd
                    yRemappedLeftEnd = yShadingLineLeftEnd
                    xRemappedRightEnd = xShadingLineRightEnd
                    yRemappedRightEnd = yShadingLineRightEnd
                    # Now, we will "remap" the end point onto the correct axis
                    # When the right end has exceeded upper-right,
                    # remap to the y-axis
                    if xShadingLineRightEnd > xs[bp][1]:
                        xRemappedRightEnd = xs[bp][1]
                        # get y value from linear algebra on modified value
                        yRemappedRightEnd \
                            = yTop + (xShadingLineRightEnd - xRemappedRightEnd)
                    # When the left end has exceeded the bottom-left,
                    # remap to the x-axis
                    if yShadingLineLeftEnd > yBottom:
                        yRemappedLeftEnd = yBottom
                        # get x value from linear algebra on modified value
                        xRemappedLeftEnd \
                            = xs[bp][0] + (yShadingLineLeftEnd
                                           - yRemappedLeftEnd)
                    # When one of the end points have reached the bottom-right
                    if xRemappedLeftEnd > xs[bp][1] \
                       or yRemappedRightEnd > yBottom:
                        break
                    # Draw the shading line
                    cv2.line(self._img,
                             (xRemappedLeftEnd, yRemappedLeftEnd),
                             (xRemappedRightEnd, yRemappedRightEnd),
                             0, 2)
            elif levels[bp] == labanotationpy.LOW:
                # Symbol is colored black
                cv2.rectangle(self._img,
                              (xs[bp][0], yTop), (xs[bp][1], yBottom),
                              0, -1)
            else:
                # Error is printed at json load for invalid levels
                pass

    # Function to draw and cut out the shape of the labanotation symbol.
    # Shape depends on the labanotation direction.
    #  @param self       The object pointer.
    #  @param directions Array of labanotation directions of a frame.
    #  @param drawFlag   Body part array indicating whether to draw the part.
    #  @param xs         Body part array indicating the draw area x-values.
    #  @param y1         Top y-value of the canvas draw area.
    #  @param y2         Bottom y-value of the canvas draw area.
    def drawAndCutOutSymbolShape(self, directions, drawFlag, xs, y1, y2):
        for bp in range(len(directions)):
            if not drawFlag[bp]:
                continue
            x1 = xs[bp][0]
            x2 = xs[bp][1]
            if directions[bp] == labanotationpy.FORWARD:
                # Symbols are L-shaped when on the right side of the score
                if xs[bp][0] > self._pixelsPerStaffSpaces*5:
                    # erase background
                    cv2.rectangle(self._img,
                                  (x1+(x2-x1)/2, y1-1), (x2+1, y1+(y2-y1)/3),
                                  255, -1)
                    # draw shape
                    pts = np.array([[x1, y1],
                                    [x1+(x2-x1)//2, y1],
                                    [x1+(x2-x1)//2, y1+(y2-y1)//3],
                                    [x2, y1+(y2-y1)//3],
                                    [x2, y2],
                                    [x1, y2]],
                                   np.int32)
                    cv2.polylines(self._img, [pts], True, 0, 2)
                # Symbols are reversed L-shaped when on the left side
                else:
                    # erase background
                    cv2.rectangle(self._img,
                                  (x1-1, y1-1), (x1+(x2-x1)//2, y1+(y2-y1)//3),
                                  255, -1)
                    # draw shape
                    pts = np.array([[x1, y1+(y2-y1)//3],
                                    [x1+(x2-x1)//2, y1+(y2-y1)//3],
                                    [x1+(x2-x1)//2, y1],
                                    [x2, y1],
                                    [x2, y2],
                                    [x1, y2]],
                                   np.int32)
                    cv2.polylines(self._img, [pts], True, 0, 2)
            elif directions[bp] == labanotationpy.RIGHT:
                # Symbols are a right-facing triangle
                # erase background
                pts = np.array([[x1, y1-1], [x2+1, y1-1], [x2+1, (y1+y2)/2]],
                               np.int32)
                cv2.fillPoly(self._img, [pts], 255)
                pts = np.array([[x1, y2+1], [x2+1, y2+1], [x2+1, (y1+y2)/2]],
                               np.int32)
                cv2.fillPoly(self._img, [pts], 255)
                # draw shape
                pts = np.array([[x1, y1], [x1, y2], [x2, (y1+y2)/2]],
                               np.int32)
                cv2.polylines(self._img, [pts], True, 0, 2)
            elif directions[bp] == labanotationpy.LEFT:
                # Symbols are a left-facing triangle
                # erase background
                pts = np.array([[x1-1, y1-1], [x2, y1-1], [x1-1, (y1+y2)/2]],
                               np.int32)
                cv2.fillPoly(self._img, [pts], 255)
                pts = np.array([[x1-1, y2+1], [x2, y2+1], [x1-1, (y1+y2)/2]],
                               np.int32)
                cv2.fillPoly(self._img, [pts], 255)
                # draw shape
                pts = np.array([[x1, (y1+y2)/2], [x2, y1], [x2, y2]],
                               np.int32)
                cv2.polylines(self._img, [pts], True, 0, 2)
            elif directions[bp] == labanotationpy.RIGHT_FORWARD:
                # Symbols are a trapezoid
                # erase background
                pts = np.array([[x1-1, y1-1],
                                [x2+1, y1-1],
                                [x1-1, y1+(y2-y1)/3]],
                               np.int32)
                cv2.fillPoly(self._img, [pts], 255)
                # draw shape
                pts = np.array([[x1, y1+(y2-y1)/3],
                                [x2, y1],
                                [x2, y2],
                                [x1, y2]],
                               np.int32)
                cv2.polylines(self._img, [pts], True, 0, 2)
            elif directions[bp] == labanotationpy.LEFT_FORWARD:
                # Symbols are a trapezoid
                # erase background
                pts = np.array([[x1, y1-1],
                                [x2+1, y1-1],
                                [x2+1, y1+(y2-y1)/3]],
                               np.int32)
                cv2.fillPoly(self._img, [pts], 255)
                pts = np.array([[x1, y1],
                                [x2, y1+(y2-y1)/3],
                                [x2, y2],
                                [x1, y2]],
                               np.int32)
                # draw shape
                cv2.polylines(self._img, [pts], True, 0, 2)
            elif directions[bp] == labanotationpy.RIGHT_BACK:
                # Symbols are a trapezoid
                # erase background
                pts = np.array([[x1-1, y2+1],
                                [x2+1, y2+1],
                                [x1-1, y2-(y2-y1)/3]],
                               np.int32)
                cv2.fillPoly(self._img, [pts], 255)
                # draw shape
                pts = np.array([[x1, y1],
                                [x2, y1],
                                [x2, y2],
                                [x1, y2-(y2-y1)/3]],
                               np.int32)
                cv2.polylines(self._img, [pts], True, 0, 2)
            elif directions[bp] == labanotationpy.LEFT_BACK:
                # Symbols are a trapezoid
                # erase background
                pts = np.array([[x1, y2+1],
                                [x2+1, y2+1],
                                [x2+1, y2-(y2-y1)/3]],
                               np.int32)
                cv2.fillPoly(self._img, [pts], 255)
                # draw shape
                pts = np.array([[x1, y1],
                                [x2, y1],
                                [x2, y2-(y2-y1)/3],
                                [x1, y2]],
                               np.int32)
                cv2.polylines(self._img, [pts], True, 0, 2)
            elif directions[bp] == labanotationpy.BACK:
                # Symbols are down L-shaped when on the right side of the score
                if xs[bp][0] > self._pixelsPerStaffSpaces*5:
                    # erase background
                    cv2.rectangle(self._img,
                                  (x1+(x2-x1)/2, y2-(y2-y1)/3), (x2+1, y2+1),
                                  255, -1)
                    # draw shape
                    pts = np.array([[x1, y1],
                                    [x2, y1],
                                    [x2, y2-(y2-y1)/3],
                                    [x1+(x2-x1)/2, y2-(y2-y1)/3],
                                    [x1+(x2-x1)/2, y2],
                                    [x1, y2]],
                                   np.int32)
                    cv2.polylines(self._img, [pts], True, 0, 2)
                # Symbols are reversed down L-shaped when on the left side
                else:
                    # erase background
                    cv2.rectangle(self._img,
                                  (x1-1, y2-(y2-y1)/3), (x1+(x2-x1)/2, y2+1),
                                  255, -1)
                    # draw shape
                    pts = np.array([[x1, y1],
                                    [x2, y1],
                                    [x2, y2],
                                    [x1+(x2-x1)/2, y2],
                                    [x1+(x2-x1)/2, y2-(y2-y1)/3],
                                    [x1, y2-(y2-y1)/3]],
                                   np.int32)
                    cv2.polylines(self._img, [pts], True, 0, 2)
            elif directions[bp] == labanotationpy.PLACE:
                # Symbols are a rectange so just draw the outer shape
                cv2.rectangle(self._img, (x1, y1), (x2, y2), 0, 2)
            else:
                # Error is printed at json load for invalid directions
                pass

    # Function to draw Labanotations onto the score
    #  @param self          The object pointer.
    #  @param labanotations The Labanotation class object to draw as a score.
    def draw(self, labanotations):

        # Add one to avoid exceeding height depending on division remains
        pixelsPerSecond \
            = int(self._height //
                  (labanotations.GetKeyframe(-1).GetTimeFromStart() + 1))
        self.initCanvas(pixelsPerSecond)

        for i in range(labanotations.Length()):
            # Get the decomposed directions and levels information
            directions, levels = labanotations.GetKeyframe(i) \
                                              .Decompose(old=True)
            # Draw flags are disabled if the same as previous frame
            drawFlag = [True for bp in
                        range(labanotationpy.NUMBER_OF_BODY_PARTS_OLD)]

            # Get the duration of the labanotation
            if i == 0:
                start_time = -90.0 / pixelsPerSecond  # ignore time frame0
                end_time = -5.0 / pixelsPerSecond  # symbol size
            else:
                start_time \
                    = labanotations.GetKeyframe(i - 1).GetTimeFromStart() \
                    - labanotations.GetKeyframe(0).GetTimeFromStart()
                end_time \
                    = labanotations.GetKeyframe(i).GetTimeFromStart() \
                    - labanotations.GetKeyframe(0).GetTimeFromStart()
                # Disable drawing if current frame is the same as the previous
                previousDirs, previousLevs = labanotations.GetKeyframe(i - 1) \
                                                          .Decompose(old=True)
                for bp in range(len(drawFlag)):
                    if directions[bp] is None \
                       or levels[bp] is None:
                        drawFlag[bp] = False
                    elif directions[bp] == previousDirs[bp] \
                         and levels[bp] == previousLevs[bp]:
                        drawFlag[bp] = False

            # Define the y-drawing area on the score
            topPadding = 3  # Padding from time
            bottomPadding = 3
            yTop = self._height \
                - int(end_time * pixelsPerSecond) + topPadding
            yBottom = self._height \
                - int(start_time * pixelsPerSecond) - bottomPadding

            # Define the x-drawing area on the score
            leftPadding = 7  # Padding from staff line
            rightPadding = 5
            xs = [None for bp in
                  range(labanotationpy.NUMBER_OF_BODY_PARTS_OLD)]
            xs[labanotationpy.mapOld[labanotationpy.HEAD]] \
                = (self._pixelsPerStaffSpaces*10 + leftPadding,
                   self._pixelsPerStaffSpaces*11 - rightPadding)
            xs[labanotationpy.mapOld[labanotationpy.RIGHT_WRIST]] \
                = (self._pixelsPerStaffSpaces*9 + leftPadding,
                   self._pixelsPerStaffSpaces*10 - rightPadding)
            xs[labanotationpy.mapOld[labanotationpy.RIGHT_ELBOW]] \
                = (self._pixelsPerStaffSpaces*8 + leftPadding,
                   self._pixelsPerStaffSpaces*9 - rightPadding)
            xs[labanotationpy.mapOld[labanotationpy.LEFT_WRIST]] \
                = (self._pixelsPerStaffSpaces*0 + leftPadding,
                   self._pixelsPerStaffSpaces*1 - rightPadding)
            xs[labanotationpy.mapOld[labanotationpy.LEFT_ELBOW]] \
                = (self._pixelsPerStaffSpaces*1 + leftPadding,
                   self._pixelsPerStaffSpaces*2 - rightPadding)

            # Draw level symbols
            # This is done by drawing a background first
            # Extra-colored areas are subtracted when the directions are drawn
            self.drawSymbolBackgroundPattern(levels,
                                             drawFlag, xs, yTop, yBottom)

            # Draw direction symbols
            self.drawAndCutOutSymbolShape(directions,
                                          drawFlag, xs, yTop, yBottom)

            # Hold time is not drawn for now

    # Function to show the canvas as a window.
    #  @param self The object pointer.
    def show(self):
        cv2.imshow('Laban', self._img)
        cv2.waitKey(0)

    # @var _numberOfStaffSpaces
    #  number of staff spaces on a labanotation score

    # @var _width
    #  width of the canvas, not scalable for now

    # @var _height
    #  height of the canvas, not scalable for now

    # @var _pageBottomMargin
    #  height of area to draw the 0th frame on a labanotation score

    # @var _img
    #  OpenCV canvas to draw the score on

    # @var _pixelsPerStaffSpaces
    #  amount of pixels between each staff space
