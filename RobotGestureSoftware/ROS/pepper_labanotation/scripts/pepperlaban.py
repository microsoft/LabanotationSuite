#!/usr/bin/env python

import naoqi
import numpy as np
import rospy


# Labanotation to joint commands converter interface for the Pepper robot.
class PepperLabanInterface:

    # The constructor.
    #  @param self The object pointer.
    #  @param ip   The IP address of Pepper.
    #  @param port The port number used by Pepper.
    def __init__(self, ip, port):
        self._ip = ip
        self._port = port

        self._timesFromStart = []
        self._keyframeJoints = []

        self._jointNames = ['HeadYaw', 'HeadPitch',
                            'LShoulderPitch', 'LShoulderRoll',
                            'LElbowYaw', 'LElbowRoll',
                            'LWristYaw', 'LHand',
                            'HipRoll', 'HipPitch', 'KneePitch',
                            'RShoulderPitch', 'RShoulderRoll',
                            'RElbowYaw', 'RElbowRoll',
                            'RWristYaw', 'RHand']

        self._jointLimit = [[np.radians(-119.5), np.radians(119.5)],  # HeadYaw
                            [np.radians(-40.5), np.radians(36.5)],  # HeadPitch
                            # calculation assumes arm down to be 0 degrees
                            # but, Pepper assumes arm forward to be 0 degrees
                            # -119.5 ~ 119.5 -> -239.0 ~ 0.0
                            [np.radians(-239.0),
                             np.radians(0.0)],  # LShoulderPitch
                            [np.radians(0.5),
                             np.radians(89.5)],  # LShoulderRoll
                            # calculation assumes bend front to be 0 degrees
                            # but, Pepper assumes bend inner to be 0 degrees
                            # in addition, rotation assumes right to left
                            # -119.5 ~ 119.5 -> -209.5 ~ 29.5
                            [np.radians(-209.5),
                             np.radians(29.5)],  # LElbowYaw
                            [np.radians(-89.5),
                             np.radians(-0.5)],  # LElbowRoll
                            [np.radians(-104.5),
                             np.radians(104.5)],  # LWristYaw
                            [0.02, 0.98],  # LHand
                            [np.radians(-29.5), np.radians(29.5)],  # HipRoll
                            [np.radians(-59.5), np.radians(59.5)],  # HipPitch
                            [np.radians(-29.5), np.radians(29.5)],  # KneePitch
                            # calculation assumes arm down to be 0 degrees
                            # but, Pepper assumes arm forward to be 0 degrees
                            # -119.5 ~ 119.5 -> -239.0 ~ 0.0
                            [np.radians(-239.0),
                             np.radians(0.0)],  # RShoulderPitch
                            [np.radians(-89.5),
                             np.radians(-0.5)],  # RShoulderRoll
                            # calculation assumes bend front to be 0 degrees
                            # but, Pepper assumes bend inner to be 0 degrees
                            # in addition, rotation assumes right to left
                            # -119.5 ~ 119.5 -> -29.5 ~ 209.5
                            [np.radians(-29.5),
                             np.radians(209.5)],  # RElbowYaw
                            # calculation assumes same as left arm
                            # 0.5 ~ 89.5 -> -89.5 ~ -0.5
                            [np.radians(-89.5),
                             np.radians(-0.5)],  # RElbowRoll
                            [np.radians(-104.5),
                             np.radians(104.5)],  # RWristYaw
                            [0.02, 0.98]]  # RHand

        # when an ip is set, setup naoqi api
        if self._ip != '':
            self._motion = naoqi.ALProxy('ALMotion', ip, port)

        # set to identy matrix as we will not be calculating torso
        self._rotMat_GL2Body = np.array([[1., 0., 0.],
                                         [0., 1., 0.],
                                         [0., 0., 1.]])

    # The method to convert ROS messages into joint angles.
    # Results will be stored as private member variables.
    #  @param self The object pointer.
    #  @param msg  The labanotation ROS message object to compile.
    def analyzeLabanotation(self, msg):
        # when an ip is set, directly call the naoqi API
        if self._ip != '':
            # below option refers to useSensors=False
            curAngles = self._motion.getAngles('Body', False)
        else:
            print("[pepper_labanotation] topic implementation not supported")
            print("[pepper_labanotation] to be supported in the future")
            return

        self._timesFromStart = []
        self._keyframeJoints = []
        # speedRatio = 1.0  # Speed resizing should be done at json layer

        # Pepper implementation parses msg as-is
        for pt in msg.points:
            print('calculating ...')
            newAngles = self.calcAnglePose(curAngles, pt)
            self._keyframeJoints.append(newAngles)
            self._timesFromStart.append(pt.time_from_start.to_sec())
            curAngles = newAngles

    # The method to send the joint angles to the Pepper robot.
    #  @param self The object pointer.
    def execute(self):
        startTime = rospy.Time.now().to_sec()
        if self._ip != '':  # use naoqi API to send the joint angles
            # input takes a list of joints with a list of positions
            ALArray = [[] for x in range(len(self._jointNames))]
            for j in range(len(self._keyframeJoints)):
                for i in range(len(self._jointNames)):
                    ALArray[i].append(self._keyframeJoints[j][i])
            # send joint angle commands
            print(self._jointNames)
            print(ALArray)
            self._motion.angleInterpolation(self._jointNames, ALArray,
                                            [self._timesFromStart]
                                            * len(self._jointNames),
                                            True)  # isAbsolute=True
        else:  # use ROS naoqi bridge to send the joint angles
            # currently not supported
            # ROS Pepper uses topic instead of actionlib, will require sleep
            # motions are not gauranteed to finish
            pass
        endTime = rospy.Time.now().to_sec()
        rospy.loginfo("PepperLaban:" + str(startTime) + ',' + str(endTime))

    # 3D rotation matrix around the X-axis.
    #  @param self  The object pointer.
    #  @param angle Angle in radians to rotate.
    def rotMat_x(self, angle):
        sinTh = np.sin(angle)
        cosTh = np.cos(angle)
        mat = np.array([[1., 0., 0.],
                        [0., cosTh, -sinTh],
                        [0., sinTh, cosTh]])
        return mat

    # 3D rotation matrix around the Y-axis.
    #  @param self  The object pointer.
    #  @param angle Angle in radians to rotate.
    def rotMat_y(self, angle):
        sinTh = np.sin(angle)
        cosTh = np.cos(angle)
        mat = np.array([[cosTh, 0., sinTh],
                        [0., 1., 0.],
                        [-sinTh, 0., cosTh]])
        return mat

    # 3D rotation matrix around the Z-axis.
    #  @param self  The object pointer.
    #  @param angle Angle in radians to rotate.
    def rotMat_z(self, angle):
        sinTh = np.sin(angle)
        cosTh = np.cos(angle)
        mat = np.array([[cosTh, -sinTh, 0.],
                        [sinTh, cosTh, 0.],
                        [0., 0., 1.]])
        return mat

    # Convert a labanotation keyframe into joint angles.
    #  @param self   The object pointer.
    #  @param angles The current joint angles of Pepper.
    #  @param pt     The labanotation keyframe (ROS message object).
    #  @return List of calculated joint angles.
    def calcAnglePose(self, angles, pt):
        # Convert ROS message to 3D arm directions (vectors).
        vec_Head = self.directionlevel2vec(pt.head)
        vec_LElbow = self.directionlevel2vec(pt.left_elbow)
        vec_RElbow = self.directionlevel2vec(pt.right_elbow)
        vec_LWrist = self.directionlevel2vec(pt.left_wrist)
        vec_RWrist = self.directionlevel2vec(pt.right_wrist)

        # Calculate the head angles
        if not pt.head.hold:
            xyz = self._rotMat_GL2Body.dot(vec_Head)
            angle_HeadYaw \
                = self.angleLimitter(self._jointNames.index('HeadYaw'),
                                     np.arctan2(xyz[1], xyz[0]))
            val = self.calcVal(np.cos(angle_HeadYaw), xyz[0],
                               np.sin(angle_HeadYaw), xyz[1])
            angle_HeadPitch \
                = self.angleLimitter(self._jointNames.index('HeadPitch'),
                                     np.arctan2(-xyz[2], val))
        else:
            angle_HeadYaw = angles[0]
            angle_HeadPitch = angles[1]

        # Calculate the left arm angles
        angle_LShoulderPitch, \
            angle_LShoulderRoll, \
            angle_LElbowYaw, \
            angle_LElbowRoll = self.solveArmKinematics(0, angles,
                                                       vec_LElbow,
                                                       vec_LWrist,
                                                       pt.left_elbow.hold,
                                                       pt.left_wrist.hold)
        # Add Pepper joint angle offsets to calculation results
        if not pt.left_elbow.hold:
            angle_LShoulderPitch += np.radians(90)
        if not pt.left_wrist.hold:
            angle_LElbowYaw += np.radians(90)
            angle_LElbowYaw *= -1

        # Calculate the right arm angles
        angle_RShoulderPitch, \
            angle_RShoulderRoll, \
            angle_RElbowYaw, \
            angle_RElbowRoll = self.solveArmKinematics(1, angles,
                                                       vec_RElbow,
                                                       vec_RWrist,
                                                       pt.right_elbow.hold,
                                                       pt.right_wrist.hold)
        # Add Pepper joint angle offsets to calculation results
        if not pt.right_elbow.hold:
            angle_RShoulderPitch += np.radians(90)
            angle_RElbowRoll *= -1
        if not pt.right_wrist.hold:
            angle_RElbowYaw -= np.radians(90)
            angle_RElbowYaw *= -1

        # Set wrist yaw to 0 for now
        angle_LWristYaw = 0
        angle_RWristYaw = 0

        return [float(angle_HeadYaw), float(angle_HeadPitch),
                angle_LShoulderPitch, angle_LShoulderRoll,
                angle_LElbowYaw, angle_LElbowRoll, angle_LWristYaw, angles[7],
                angles[8], angles[9], angles[10],
                angle_RShoulderPitch, angle_RShoulderRoll,
                angle_RElbowYaw, angle_RElbowRoll, angle_RWristYaw, angles[16]]

    # Return angles so that they are within angle limits.
    #  @param self    The object pointer.
    #  @param jointId The joint to check.
    #  @param angle   The current angle set for the joint.
    #  @return The angle within the limits.
    def angleLimitter(self, jointId, angle):
        angleMin, angleMax = self._jointLimit[jointId]
        if angle < angleMin:
            print("WARNING following angle exceeds limit value!: %s %f -> %f"
                  % (self._jointNames[jointId],
                     np.degrees(angle), np.degrees(angleMin)))
            return angleMin
        elif angle > angleMax:
            print("WARNING following angle exceeds limit value!: %s %f -> %f"
                  % (self._jointNames[jointId],
                     np.degrees(angle), np.degrees(angleMax)))
            return angleMax
        return angle

    # Convert labanotation direction and level to vectors in 3D space.
    #  @param self The object pointer.
    #  @param d    The labanotation (ROS message object).
    #  @return Converted vector in 3D space.
    def directionlevel2vec(self, d):
        if d.place:
            x = 0
            y = 0
        else:
            # multiply np.cos(d.level) to normalize
            # to avoid float errors, do not divide by norm
            x = abs(np.cos(d.level)) * np.cos(d.direction)
            y = abs(np.cos(d.level)) * np.sin(d.direction)
        z = np.sin(d.level)
        return np.array([[x], [y], [z]])

    # Function to determine the appropriate arctan argument value.
    #  @param self The object pointer.
    #  @param tx   Target x value.
    #  @param x    Reference x value to consider appropriateness.
    #  @param ty   Target y value.
    #  @param y    Reference y value to consider appropriateness.
    #  @return An appropriate arctan argument value.
    def calcVal(self, tx, x, ty, y):
        val = np.sqrt(x**2 + y**2)
        if abs(tx * val) > abs(ty * y):
            if tx * x > 0:
                return val
            else:
                return -val
        else:
            if ty * y > 0:
                return val
            else:
                return -val

    # Function to convert calculation results to the appropriate data type.
    #  @param self The object Pointer.
    #  @param v    The calculated result.
    #  @return The converted result.
    def optimize(self, v):
        # make sure to handle float errors
        if abs(v[0][0]) < 0.01:
            v[0][0] = 0
        if abs(v[1][0]) < 0.01:
            v[1][0] = 0
        if abs(v[2][0]) < 0.01:
            v[2][0] = 0
        # array(array()) -> array()
        return np.array([v[0][0], v[1][0], v[2][0]], dtype='float')

    # Function to convert arm directions to Pepper joint angles.
    #  @param self      The object pointer.
    #  @param arm       The arm to calculate.
    #  @param angles    The current joint angles.
    #  @param vecElbow  The upper arm direction in 3D space
    #  @param vecWrist  The lower arm direction in 3D space.
    #  @param holdUpper True if upper arm values are same as previous frame.
    #  @param holdLower True if lower arm values are same as previous frame.
    #  @return The calculated joint angles.
    def solveArmKinematics(self, arm, angles, vecElbow, vecWrist,
                           holdUpper, holdLower):

        ids = [[2, 3, 4, 5], [11, 12, 13, 14]]

        # Calculate upper arm
        xyz = self._rotMat_GL2Body.dot(vecElbow)
        angle_ShoulderPitch = self.angleLimitter(ids[arm][0],
                                                 np.arctan2(-xyz[0], -xyz[2]))
        val = self.calcVal(-np.sin(angle_ShoulderPitch), xyz[0],
                           -np.cos(angle_ShoulderPitch), xyz[2])
        angle_ShoulderRoll = self.angleLimitter(ids[arm][1],
                                                np.arctan2(xyz[1], val))

        # Calculate lower arm
        intMat = self.rotMat_x(-angle_ShoulderRoll) \
                     .dot(self.rotMat_y(-angle_ShoulderPitch))
        xyz = intMat.dot(self._rotMat_GL2Body.dot(vecWrist))
        xyz = self.optimize(xyz)
        angle_ElbowYaw = self.angleLimitter(ids[arm][2],
                                            np.arctan2(xyz[1], xyz[0]))
        val = self.calcVal(-np.cos(angle_ElbowYaw), xyz[0],
                           -np.sin(angle_ElbowYaw), xyz[1])
        angle_ElbowRoll = self.angleLimitter(ids[arm][3],
                                             np.arctan2(val, -xyz[2]))

        if holdUpper:
            angle_ShoulderPitch = angles[ids[arm][0]]
            angle_ShoulderRoll = angles[ids[arm][1]]
        if holdLower:
            angle_ElbowYaw = angles[ids[arm][2]]
            angle_ElbowRoll = angles[ids[arm][3]]

        return float(angle_ShoulderPitch), float(angle_ShoulderRoll), \
            float(angle_ElbowYaw), float(angle_ElbowRoll)

    # @var _ip
    #   IP address string to connect Pepper

    # @var _port
    #   port number to connect Pepper

    # @var _timesFromStart
    #   list of keyframe starting times

    # @var _keyframeJoints
    #   list of list of joints

    # @var _jointNames
    #   defined Pepper joint names used by naoqi

    # @var _jointLimit
    #   converted limit of Pepper's joint angles for kinematics calculation

    # @var _rotMat_GL2Body
    #   matrix to hold the pose of Pepper's torso (identy for now)
