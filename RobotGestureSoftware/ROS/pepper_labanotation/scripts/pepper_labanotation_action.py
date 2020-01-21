#!/usr/bin/env python

# A ROS node to execute labanotation ROS messages as robot commands.

import rospy
import pepperlaban
import labanotation_msgs.msg
import actionlib
import naoqi


# Class to execute labanotation ROS messages as robot commands.
class PepperLabanotationAction(object):

    # The constructor.
    #  @param ip   The IP address of Pepper.
    #  @param port The port number used by Pepper.
    def __init__(self, ip, port):
        # create the interface to convert labanotation messages to commands
        self.labanInterface = pepperlaban.PepperLabanInterface(ip, port)

        # server to execute labanotation msgs as robot commands
        self._as = actionlib.SimpleActionServer(
            '~labanotation_action',
            labanotation_msgs.msg.LabanotationAction,
            execute_cb=self.execute_cb, auto_start=False)

        # start the command server
        self._as.start()

    # A callback to handle incoming labanotation ROS messages,
    #  @param self The object pointer.
    #  @param goal A ROS msg containing labanotation keyframes.
    def execute_cb(self, goal):
        # store the message to command
        self.labanInterface.analyzeLabanotation(goal.trajectory)

        self.labanInterface.execute()  # execute the command

        # tell the command requester that the execution was successful
        result = labanotation_msgs.msg.LabanotationResult()
        result.error_code = labanotation_msgs.msg.LabanotationResult.SUCCESSFUL
        result.error_string = ""
        self._as.set_succeeded(result)


if __name__ == "__main__":
    rospy.init_node('pepper_labanotation')

    # get IP and port information from ROS parameter server
    pepperIP = rospy.get_param("~nao_ip", '')
    pepperPort = rospy.get_param("~nao_port", 0)

    # to run this node, autonomous life must be disabled
    al = naoqi.ALProxy('ALAutonomousLife', pepperIP, pepperPort)
    if al.getState() != 'disabled':
        al.setState('disabled')
        rospy.sleep(1.)
    al = naoqi.ALProxy('ALMotion', pepperIP, pepperPort)
    if not al.robotIsWakeUp():
        al.wakeUp()
        rospy.sleep(1.)

    # create the server
    server = PepperLabanotationAction(pepperIP, pepperPort)

    rospy.spin()
