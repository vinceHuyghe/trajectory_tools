#!/usr/bin/env python3
from math import pi

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion
from move_group_sequence.move_group_sequence import Circ, Lin, Ptp, from_euler
from trajectory_tools.srv import RandomPose
from trajectory_tools.trajectory_handler import TrajectoryHandler


def robot_program():

    rospy.wait_for_service("/random_pose")
    rospy.loginfo("Waiting for /random_pose service")

    th = TrajectoryHandler()

    # go to start
    start = th.start_position

    th.sequencer.plan(Ptp(goal=start))
    th.sequencer.execute()

    pose = Pose(position=Point(0.6, 0, 0.5), orientation=Quaternion(0.0, 0.5, 0.0, 0.5))

    th.sequencer.plan(Ptp(goal=pose))
    th.sequencer.execute()

    # Start client loop
    random_offset = rospy.ServiceProxy("/random_pose", RandomPose)

    while not rospy.is_shutdown():

        resp = random_offset(pose)

        th.sequencer.plan(Lin(goal=resp.pose, vel_scale=0.3, acc_scale=0.1))
        th.sequencer.execute()


if __name__ == "__main__":

    robot_program()
