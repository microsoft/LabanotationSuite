#!/usr/bin/env python

# Converts a loaded JSON file into a ROS message.

import labanotationpy
import labanotationpy.loader

import rospy
import labanotation_msgs.msg

from labanotation_msgs.msg import LabanotationDirectionSymbol


# Map to convert JSON 'direction' keys to ROS message 'direction' keys
directionMap = {labanotationpy.FORWARD: LabanotationDirectionSymbol.FORWARD,

                labanotationpy.LEFT_FORWARD:
                LabanotationDirectionSymbol.LEFT_FORWARD,

                labanotationpy.LEFT: LabanotationDirectionSymbol.LEFT,

                labanotationpy.LEFT_BACK:
                LabanotationDirectionSymbol.LEFT_BACK,

                labanotationpy.BACK: LabanotationDirectionSymbol.BACK,

                labanotationpy.RIGHT_FORWARD:
                LabanotationDirectionSymbol.RIGHT_FORWARD,

                labanotationpy.RIGHT: LabanotationDirectionSymbol.RIGHT,

                labanotationpy.RIGHT_BACK:
                LabanotationDirectionSymbol.RIGHT_BACK}

# Map to convert JSON 'level' keys to ROS message 'level' keys
levelMap = {labanotationpy.HIGH: LabanotationDirectionSymbol.HIGH,
            labanotationpy.MIDDLE: LabanotationDirectionSymbol.MIDDLE,
            labanotationpy.LOW: LabanotationDirectionSymbol.LOW}


# Convert a pyobject with body symbol information to a ROS message.
#  @param body Pyobject with body (e.g. head) symbol information.
#  @param hold Boolean true if copying from the previous frame.
#  @return Converted ROS message (body symbol) object.
def py2msg(body, hold):
    ret = labanotation_msgs.msg.LabanotationDirectionSymbol()
    if hold:  # body uses the same symbol as the previous frame
        ret.hold = True
        ret.place = False
        ret.direction = 0.0
        ret.level = 0.0
    else:  # body has a new symbol for this frame
        ret.hold = False
        ret.place = (body.GetDirection() == labanotationpy.PLACE)
        if not ret.place:
            ret.direction = directionMap[body.GetDirection()]
        ret.level = levelMap[body.GetLevel()]
    return ret


# Load a pyobject from a JSON file name and convert to a ROS msg.
#  @param jsonfilename Name of the JSON file to load.
#  @return Converted ROS message (full labanotation score) object.
def jsonfile2msg(jsonfilename):
    if len(jsonfilename.split('/')) < 1:
        rospy.logwarn('filename must be passed with absolute path')

    try:
        lpy = labanotationpy.loader.LoadJsonFile(jsonfilename,
                                                 old=True)
    except Exception:
        return labanotation_msgs.msg.LabanotationTrajectory()

    lt = labanotation_msgs.msg.LabanotationTrajectory()
    lt.header.stamp = rospy.Time.now()

    for i in range(lpy.Length()):
        kf = lpy.GetKeyframe(i)
        pt = labanotation_msgs.msg.LabanotationTrajectoryPoint()

        pt.time_from_start = rospy.Duration(kf.GetTimeFromStart())
        head = kf.GetBodyPart(labanotationpy.HEAD)
        pt.head = py2msg(head, head.isEmpty())
        rightElbow = kf.GetBodyPart(labanotationpy.RIGHT_ELBOW)
        pt.right_elbow = py2msg(rightElbow, rightElbow.isEmpty())
        rightWrist = kf.GetBodyPart(labanotationpy.RIGHT_WRIST)
        pt.right_wrist = py2msg(rightWrist, rightWrist.isEmpty())
        leftElbow = kf.GetBodyPart(labanotationpy.LEFT_ELBOW)
        pt.left_elbow = py2msg(leftElbow, leftElbow.isEmpty())
        leftWrist = kf.GetBodyPart(labanotationpy.LEFT_WRIST)
        pt.left_wrist = py2msg(leftWrist, leftWrist.isEmpty())

        lt.points.append(pt)

        # add an extra labanotation point if keyframe has hold
        if kf.GetHoldDuration() > 0:
            pth = labanotation_msgs.msg.LabanotationTrajectoryPoint()
            pth.time_from_start \
                = rospy.Duration(kf.GetTimeFromStart()
                                 + kf.GetHoldDuration())
            pth.head = py2msg(None, True)
            pth.right_elbow = py2msg(None, True)
            pth.right_wrist = py2msg(None, True)
            pth.left_elbow = py2msg(None, True)
            pth.left_wrist = py2msg(None, True)
            lt.points.append(pth)

    return lt
