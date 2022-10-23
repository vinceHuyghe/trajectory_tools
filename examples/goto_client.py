#! /usr/bin/env python3

import actionlib
import rospy
import trajectory_tools.msg
from geometry_msgs.msg import Point, Pose, Quaternion


def goto_client():

    # Creates the SimpleActionClient, passing the type of the action
    # (GotoAction) to the constructor.
    client = actionlib.SimpleActionClient(
        "/goto_action_server_example", trajectory_tools.msg.GotoAction
    )

    # Waits until the action server has started
    client.wait_for_server()

    # Creates a goal to send to the action server.
    pose = Pose(position=Point(1, 0.0, 0.6), orientation=Quaternion(0.5, 0.5, 0.5, 0.5))

    goal = trajectory_tools.msg.GotoGoal(pose)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()


if __name__ == "__main__":

    rospy.init_node("action_client")
    result = goto_client()
    rospy.loginfo(f"action result: {result.goal_reached}")
