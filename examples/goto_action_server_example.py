#!/usr/bin/env python3

from math import pi

import actionlib
import rospy
import trajectory_tools.msg
from move_group_sequence.move_group_sequence import Circ, Lin, Ptp, from_euler
from trajectory_tools.trajectory_handler import TrajectoryHandler


class GotoAction:

    # create messages that are used to publish feedback/result
    _feedback = trajectory_tools.msg.GotoFeedback()
    _result = trajectory_tools.msg.GotoResult()

    def __init__(self):

        self._th = TrajectoryHandler()
        self._as = actionlib.SimpleActionServer(
            self._th.name,
            trajectory_tools.msg.GotoAction,
            execute_cb=self.execute_callback,
            auto_start=False,
        )
        self._as.start()

    def execute_callback(self, pose):

        rospy.loginfo(f"{self._th.name}: exec_callback")
        # start executing the action
        plan = self._th.sequencer.plan(
            Ptp(goal=pose.goal, vel_scale=0.3, acc_scale=0.3)
        )
        self._feedback.planning_succeeded = plan[0]

        # publish the feedback
        self._as.publish_feedback(self._feedback)

        # check that preempt has not been requested by the client
        # in this specific use case the preempt is not required, as
        # there is only one goal to execute. It is in this example for
        # completeness 
        if self._as.is_preempt_requested():
            self._as.set_preempted()
            return rospy.loginfo(f"{self._th.name}: Action Preempted")

        # if execute returns None motion was executed
        if not self._th.sequencer.execute():
            self._result.goal_reached = True
            self._as.set_succeeded(self._result)
            return rospy.loginfo(f"{self._th.name}: Action Succeeded")

        else:
            return rospy.loginfo(f"{self._th.name}: Action Failed")


if __name__ == "__main__":

    GotoAction()
    rospy.spin()
