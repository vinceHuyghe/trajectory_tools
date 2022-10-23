#!/usr/bin/env python3
from math import pi

import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from trajectory_tools.trajectory_handler import TrajectoryHandler
from move_group_sequence.move_group_sequence import Circ, Lin, Ptp, from_euler


def robot_program():

    ee_name = "extruder"

    pose0 = Pose(
        position=Point(0.8, 0, 0.3), orientation=Quaternion(0.0, 1.0, 0.0, 0.0)
    )
    pose1 = Pose(
        position=Point(0.8, 0.6, 0.3), orientation=Quaternion(0.0, 1.0, 0.0, 0.0)
    )
    pose2 = Pose(
        position=Point(0.8, -0.6, 0.3), orientation=Quaternion(0.0, 1.0, 0.0, 0.0)
    )

    poses = [pose0, pose1, pose0, pose2]

    th = TrajectoryHandler()

    # display pose markers in rviz
    th.publish_marker_array(poses)

    # attach collision object with subframe
    th.attach_end_effector(ee_name)
    th.move_group.set_end_effector_link(f"{ee_name}/tcp")
    rospy.loginfo(
        f"{th.name}: end effector link set to {th.move_group.get_end_effector_link()}"
    )

    # set planning pipeline
    th.move_group.set_planning_pipeline_id("ompl")

    th.move_group.set_named_target("start_ee")
    th.move_group.go()

    for pose in poses:
        th.move_group.set_pose_target(pose)
        success, plan = th.move_group.plan()[:2]
        if success:
            th.move_group.execute(plan, wait=True)
        else:
            return rospy.loginfo(f"{th.name}: planning failed, robot program aborted")

    for pose in poses:
        plan = th.sequencer.plan(Lin(goal=pose, vel_scale=0.3, acc_scale=0.3))
        th.sequencer.execute()


if __name__ == "__main__":

    robot_program()
