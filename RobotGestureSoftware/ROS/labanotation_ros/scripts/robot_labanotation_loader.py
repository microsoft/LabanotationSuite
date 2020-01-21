#!/usr/bin/env python

# A ROS node to publish labanotation msgs from a specified JSON file name.

import rospy
import labanotation_msgs.msg
import jsonfile2msg
import actionlib


# Class to publish labanotation as a ROS action from incoming JSON file names.
class Jsonfile2MsgActionServer(object):

    # The constructor.
    def __init__(self):
        # server to listen to incoming msgs with JSON file names
        self._as = actionlib.SimpleActionServer(
            '~json_action',
            labanotation_msgs.msg.LabanotationFromJsonFileAction,
            execute_cb=self.execute_cb, auto_start=False)

        # client to request labanotation msgs
        self._client = actionlib.SimpleActionClient(
            '/' + rospy.get_param('~namespace', '') + '/labanotation_action',
            labanotation_msgs.msg.LabanotationAction)
        self._client.wait_for_server()

        # start the listening server
        self._as.start()

    # A callback to handle incoming JSON file names.
    #  @param self The object pointer.
    #  @param goal A ROS msg containing JSON file name information.
    def execute_cb(self, goal):
        startTime = rospy.Time.now().to_sec()  # for logging

        # parse and create a labanotation ROS action
        msg = labanotation_msgs.msg.LabanotationGoal(
            trajectory=jsonfile2msg.jsonfile2msg(goal.filename))

        # create a result object to send back to the JSON file name sender
        result = labanotation_msgs.msg.LabanotationFromJsonFileResult()

        # stream the labanotation msg only if 'execute' is true
        if goal.execute:
            self._client.send_goal(msg)
            self._client.wait_for_result()  # wait for execution to finish
        else:  # else, store the parsed result
            result.trajectory = msg.trajectory

        # tell the JSON requester that the execution(/parsing) was successful
        result.error_code = labanotation_msgs.msg.LabanotationResult.SUCCESSFUL
        result.error_string = ""
        self._as.set_succeeded(result)

        endTime = rospy.Time.now().to_sec()  # for logging
        rospy.loginfo("LabanotationAction:"
                      + str(startTime) + ',' + str(endTime)
                      + ',' + goal.filename)


if __name__ == "__main__":
    rospy.init_node('robot_labanotation_loader')

    server = Jsonfile2MsgActionServer()

    rospy.spin()
