#!/usr/bin/env python

# @package docstring
# Provide explanation of this code here.
# This code should be independent from ROS.
# opencv codes should go to draw.py.
# matplotlib codes should go to plot.py
#
# More Details.
# Disclaimer and liscensing.
# Author information and email address to contact.
# Date of release.

import numpy as np
import matplotlib.pyplot as pl
import mpl_toolkits.mplot3d
import labanotationpy


# Class to plot a stick figure from a labanotation keyframe object.
class StickFigurePlot3D:

    # The constructor.
    def __init__(self):

        # Initiate plots
        self._fig = pl.figure()
        self._ax = mpl_toolkits.mplot3d.Axes3D(self._fig)
        self._ax.clear()
        self._ax.view_init(elev=20, azim=-135)
        self._ax.set_xlim3d(-50, 50)
        self._ax.set_ylim3d(-50, 50)
        self._ax.set_zlim3d(-50, 50)
        self._ax.text(-90, 0, 90, '', fontsize=30)

        # Initiate stick figures
        # [parent joint, x, y, z] serial number is the same as Kinect
        self._joints = []
        self._joints.append([-1, 0, 0, 0])  # spineBase, #0
        self._joints.append([0, 0, 0, 0])  # spineMid, #1
        self._joints.append([20, 0, 0, 0])  # neck,#2
        self._joints.append([2, 0, 0, 0])  # head, #3
        self._joints.append([20, 0, 0, 0])  # shoulderLeft, #4
        self._joints.append([4, 0, 0, 0])  # elbowLeft, #5
        self._joints.append([5, 0, 0, 0])  # wristLeft, #6
        self._joints.append([6, 0, 0, 0])  # handLeft, #7
        self._joints.append([20, 0, 0, 0])  # shoulderRight, #8
        self._joints.append([8, 0, 0, 0])  # elbowRight, #9
        self._joints.append([9, 0, 0, 0])  # wristRight, #10
        self._joints.append([10, 0, 0, 0])  # handRight, #11
        self._joints.append([0, 0, 0, 0])  # hipLeft, #12
        self._joints.append([12, 0, 0, 0])  # kneeLeft, #13
        self._joints.append([13, 0, 0, 0])  # ankleLeft, #14
        self._joints.append([14, 0, 0, 0])  # footLeft, #15
        self._joints.append([0, 0, 0, 0])  # hipRight, #16
        self._joints.append([16, 0, 0, 0])  # kneeRight, #17
        self._joints.append([17, 0, 0, 0])  # ankleRight, #18
        self._joints.append([18, 0, 0, 0])  # footRight, #19
        self._joints.append([1, 0, 0, 0])  # spineSoulder, #20
        self._joints.append([7, 0, 0, 0])  # handTipLeft, #21
        self._joints.append([6, 0, 0, 0])  # thumbLeft, #22
        self._joints.append([11, 0, 0, 0])  # handTipRight, #23
        self._joints.append([10, 0, 0, 0])  # thumbTipRight, #24

    # Decomposed LabanotationBodyParts to a 3D space vector.
    #  @param self       The object pointer.
    #  @param directions Array of labanotation directions.
    #  @param levels     Array of labanotation levels.
    def decomposedBodyParts2Vec(self, directions, levels):
        vecs = [[0, 0, 0] for i in range(len(directions))]
        for i in range(len(directions)):
            if levels[i] == labanotationpy.HIGH:
                theta = 45
            elif levels[i] == labanotationpy.MIDDLE:
                theta = 90
            elif levels[i] == labanotationpy.LOW:
                theta = 135
            else:  # unexpected
                # Error is printed at json load for invalid levels
                theta = 180

            if directions[i] == labanotationpy.FORWARD:
                phi = 0
            elif directions[i] == labanotationpy.RIGHT_FORWARD:
                phi = -45
            elif directions[i] == labanotationpy.RIGHT:
                phi = -90
            elif directions[i] == labanotationpy.RIGHT_BACK:
                phi = -135
            elif directions[i] == labanotationpy.BACK:
                phi = 180
            elif directions[i] == labanotationpy.LEFT_BACK:
                phi = 135
            elif directions[i] == labanotationpy.LEFT:
                phi = 90
            elif directions[i] == labanotationpy.LEFT_FORWARD:
                phi = 45
            elif directions[i] == labanotationpy.PLACE:
                if levels[i] == labanotationpy.HIGH:
                    if i == labanotationpy.mapOld[
                            labanotationpy.LEFT_ELBOW]:
                        theta = 10
                        phi = 90
                    elif i == labanotationpy.mapOld[
                            labanotationpy.RIGHT_ELBOW]:
                        theta = 10
                        phi = -90
                    elif (i == labanotationpy.mapOld[
                            labanotationpy.LEFT_WRIST]
                          or i == labanotationpy.mapOld[
                              labanotationpy.RIGHT_WRIST]):
                        theta = 0
                        phi = 0
                    else:
                        theta = 0
                        phi = 0
                elif levels[i] == labanotationpy.LOW:
                    if i == labanotationpy.mapOld[
                            labanotationpy.LEFT_ELBOW] \
                       or i == labanotationpy.mapOld[
                           labanotationpy.RIGHT_ELBOW]:
                        theta = 170
                        phi = 180
                    elif (i == labanotationpy.mapOld[
                            labanotationpy.LEFT_WRIST]
                          or i == labanotationpy.mapOld[
                              labanotationpy.RIGHT_WRIST]):
                        theta = 170
                        phi = 0
                    else:
                        theta = 0
                        phi = 0
                else:  # unexpected
                    # Error is printed at json load for invalid levels
                    theta = 180
                    phi = 0
            else:  # unexpected
                # Error is printed at json load for invalid directions
                phi = 0
            vecs[i] = [20*np.sin(np.deg2rad(theta))*np.sin(np.deg2rad(phi)),
                       -20*np.sin(np.deg2rad(theta))*np.cos(np.deg2rad(phi)),
                       20*np.cos(np.deg2rad(theta))]
        return vecs

    # Calculate the position of the target joint.
    #  @param self The object pointer
    #  @param i    The target joint.
    #  @param vec  The direction of the joint from its parent joint.
    def calcStickFigureJoint(self, i, vec):
        self._joints[i][1] = self._joints[self._joints[i][0]][1] + vec[0]
        self._joints[i][2] = self._joints[self._joints[i][0]][2] + vec[1]
        self._joints[i][3] = self._joints[self._joints[i][0]][3] + vec[2]

    # Calculate all stick figure joint positions.
    #  @param self          The object pointer.
    #  @param labanotation  LabanotationKeyframe object to calculate from.
    def calcStickFigureJoints(self, labanotation):
        directions, levels = labanotation.Decompose(old=True)
        vecs = self.decomposedBodyParts2Vec(directions, levels)

        # spineBase->spineMid
        self.calcStickFigureJoint(1, [0, 1, 10])
        # spineMid->spineShoulder
        self.calcStickFigureJoint(20, [0, 0, 22])
        # spineShoulder->neck
        self.calcStickFigureJoint(2, [0, 0, 7])
        # neck->head
        self.calcStickFigureJoint(3, [0, -2, 12])

        # spineShoulder->shoulderLeft
        self.calcStickFigureJoint(4, [10, 0, -2])
        # shoulderLeft->elbowLeft
        self.calcStickFigureJoint(
            5,
            vecs[labanotationpy.mapOld[labanotationpy.LEFT_ELBOW]])
        # elbowLeft->wristLeft
        self.calcStickFigureJoint(
            6,
            vecs[labanotationpy.mapOld[labanotationpy.LEFT_WRIST]])
        # wristLeft->handLeft
        self.calcStickFigureJoint(7, [0, 0, 0])
        # spineShoulder->shoulderRight
        self.calcStickFigureJoint(8, [-10, 0, -2])
        # shoulderRight->elbowRight
        self.calcStickFigureJoint(
            9,
            vecs[labanotationpy.mapOld[labanotationpy.RIGHT_ELBOW]])
        # elbowRight->wristRight
        self.calcStickFigureJoint(
            10,
            vecs[labanotationpy.mapOld[labanotationpy.RIGHT_WRIST]])
        # wristLeft->handLeft
        self.calcStickFigureJoint(11, [0, 0, 0])

        # spineBase->hipLeft
        self.calcStickFigureJoint(12, [5, 0, -5])
        # hipLeft->kneeLeft
        self.calcStickFigureJoint(13, [2, 0, -25])
        # kneeLeft->ankleLeft
        self.calcStickFigureJoint(14, [0, 4, -25])
        # ankleLeft->footLeft
        self.calcStickFigureJoint(15, [3, -7, -1])
        # spineBase->hipRight
        self.calcStickFigureJoint(16, [-5, 0, -5])
        # hipRight->kneeRight
        self.calcStickFigureJoint(17, [-2, 0, -25])
        # kneeRight->ankleRight
        self.calcStickFigureJoint(18, [0, 4, -25])
        # ankleRight->footRight
        self.calcStickFigureJoint(19, [-3, -7, -1])

    # Function to draw all joints on the plot.
    #  @param self          The object pointer.
    def drawJoints(self):
        for j in self._joints:
            # front view
            self._ax.scatter(j[1]-50, j[2], j[3], color='k', s=50)
            # side view
            self._ax.scatter(j[2]+50, -j[1], j[3], color='k', s=50)

    # Function to draw all limbs on the plot
    #  @param self          The object pointer.
    def drawLimbs(self):
        # most limbs connect between parent and child joint
        for i in range(1, len(self._joints)-4):
            # front view
            a = self._joints[i]
            start = [a[1]-50, a[2], a[3]]
            b = self._joints[self._joints[i][0]]
            end = [b[1]-50, b[2], b[3]]
            self._ax.plot([start[0], end[0]],
                          [start[1], end[1]],
                          [start[2], end[2]], color='k', linewidth='2.5')
            # side view
            a = self._joints[i]
            start = [a[2]+50, -a[1], a[3]]
            b = self._joints[self._joints[i][0]]
            end = [b[2]+50, -b[1], b[3]]
            self._ax.plot([start[0], end[0]],
                          [start[1], end[1]],
                          [start[2], end[2]], color='k', linewidth='2.5')

        # spine to left, right shoulders (front)
        a = self._joints[1]
        start = [a[1]-50, a[2], a[3]]
        b = self._joints[4]
        end = [b[1]-50, b[2], b[3]]
        self._ax.plot([start[0], end[0]],
                      [start[1], end[1]],
                      [start[2], end[2]], color='k', linewidth='2.5')
        b = self._joints[8]
        end = [b[1]-50, b[2], b[3]]
        self._ax.plot([start[0], end[0]],
                      [start[1], end[1]],
                      [start[2], end[2]], color='k', linewidth='2.5')

        # spine to left, right shoulders (side)
        a = self._joints[1]
        start = [a[2]+50, -a[1], a[3]]
        b = self._joints[4]
        end = [b[2]+50, -b[1], b[3]]
        self._ax.plot([start[0], end[0]],
                      [start[1], end[1]],
                      [start[2], end[2]], color='k', linewidth='2.5')
        b = self._joints[8]
        end = [b[2]+50, -b[1], b[3]]
        self._ax.plot([start[0], end[0]],
                      [start[1], end[1]],
                      [start[2], end[2]], color='k', linewidth='2.5')

    # Function to plot Labanotation keyframe onto 3D space.
    #  @param self          The object pointer.
    #  @param labanotation  The Labanotation keyframe object to plot.
    def plot(self, labanotation):
        self.calcStickFigureJoints(labanotation)
        self._ax.clear()
        self._ax.view_init(elev=20, azim=-135)
        self._ax.set_xlim3d(-50, 50)
        self._ax.set_ylim3d(-50, 50)
        self._ax.set_zlim3d(-50, 50)
        self._ax.text(-90, 0, 90, '', fontsize=30)
        self.drawJoints()
        self.drawLimbs()
        self._fig.canvas.draw()

    # Function to show the canvas as a window.
    #  @param self The object pointer.
    def show(self):
        pl.show()
