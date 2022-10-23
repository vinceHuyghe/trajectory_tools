#!/usr/bin/env python3

from math import pi

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion
from move_group_sequence.move_group_sequence import Circ, Lin, Ptp, from_euler
from trajectory_tools.trajectory_handler import TrajectoryHandler


def robot_program():

    th = TrajectoryHandler(sim=False)

    start = th.start_position
    pose_l = Pose(position=Point(0.8, -0.6, 0.4), orientation=from_euler(0.0, pi, 0.0))
    pose_r = Pose(position=Point(0.8, 0.6, 0.4), orientation=from_euler(0.0, pi, 0.0))

    th.sequencer.plan(Ptp(goal=start, vel_scale=0.3, acc_scale=0.3))
    th.sequencer.execute()

    th.sequencer.plan(Ptp(goal=pose_l, vel_scale=0.3, acc_scale=0.3))
    th.sequencer.execute()

    th.set_io(1, 0, 1)

    th.sequencer.plan(Ptp(goal=pose_r, vel_scale=0.3, acc_scale=0.3))
    th.sequencer.execute()

    th.set_io(1, 0, 0)


if __name__ == "__main__":

    robot_program()
