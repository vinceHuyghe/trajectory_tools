#!/usr/bin/env python3

from math import pi

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion
from move_group_sequence.move_group_sequence import Circ, Lin, Ptp, Sequence, from_euler
from trajectory_tools.trajectory_handler import TrajectoryHandler


def robot_program():

    th = TrajectoryHandler()

    th.sequencer.plan(Ptp(goal=th.start))
    th.sequencer.execute()

    sequence = Sequence()

    sequence.append(Ptp(goal=th.start))
    sequence.append(
        Ptp(
            goal=Pose(
                position=Point(0.4, 0.0, 0.6),
                orientation=Quaternion(0.5, 0.5, 0.5, 0.5),
            ),
            vel_scale=0.1,
            acc_scale=0.05,
        )
    )

    sequence.append(
        Lin(
            goal=Pose(
                position=Point(0.4, 0.3, 0.6),
                orientation=Quaternion(0.5, 0.5, 0.5, 0.5),
            ),
            vel_scale=0.1,
            acc_scale=0.05,
        ),
        blend_radius=0.01,
    )

    sequence.append(
        Lin(
            goal=Pose(
                position=Point(0.4, -0.3, 0.6),
                orientation=Quaternion(0.5, 0.5, 0.5, 0.5),
            ),
            vel_scale=0.1,
            acc_scale=0.05,
        )
    )

    sequence.append(
        Circ(
            goal=Pose(
                position=Point(0.4, 0.0, 0.9),
                orientation=Quaternion(0.5, 0.5, 0.5, 0.5),
            ),
            center=Point(0.4, 0.0, 0.6),
        ),
        blend_radius=0.01,
    )

    sequence.append(
        Circ(
            goal=Pose(
                position=Point(0.4, 0.3, 0.6),
                orientation=Quaternion(0.5, 0.5, 0.5, 0.5),
            ),
            center=Point(0.4, 0.0, 0.6),
        ),
        blend_radius=0.01,
    )

    sequence.append(
        Lin(
            goal=Pose(
                position=Point(0.4, 0.0, 0.6),
                orientation=Quaternion(0.5, 0.5, 0.5, 0.5),
            ),
            vel_scale=0.1,
            acc_scale=0.05,
        )
    )

    th.sequencer.plan(sequence)

    th.display_trajectory()

    th.sequencer.execute()


if __name__ == "__main__":

    robot_program()
