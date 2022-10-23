#!/usr/bin/env python3

from math import pi

from geometry_msgs.msg import Point, Pose, Quaternion
from move_group_sequence.move_group_sequence import Circ, Lin, Ptp, from_euler
from trajectory_tools.trajectory_handler import TrajectoryHandler


def robot_program():

    th = TrajectoryHandler()

    start = th.start_position
    pose_l = Pose(position=Point(0.6, -0.6, 0.4),
                  orientation=from_euler(0.0, pi, 0.0))
    pose_r = Pose(position=Point(0.6, 0.6, 0.4),
                  orientation=from_euler(0.0, pi, 0.0))

    poses = [start, pose_l, pose_r]

    th.publish_marker_array([pose_l, pose_r])

    for pose in poses:
        th.sequencer.plan(Ptp(goal=pose, vel_scale=0.3, acc_scale=0.3))
        th.sequencer.execute()


if __name__ == '__main__':

    robot_program()
