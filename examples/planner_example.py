#!/usr/bin/env python3
from math import pi

import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import Mesh, MeshTriangle, Plane, SolidPrimitive
from std_msgs.msg import Header
from trajectory_tools.trajectory_handler import TrajectoryHandler


def robot_program():

    pose0 = Pose(
        position=Point(0.8, -0.6, 0.3), orientation=Quaternion(0.0, 1.0, 0.0, 0.0)
    )
    pose1 = Pose(
        position=Point(0.8, 0.6, 0.3), orientation=Quaternion(0.0, 1.0, 0.0, 0.0)
    )

    poses = [pose0, pose1]

    th = TrajectoryHandler()

    # display pose markers in rviz
    th.publish_marker_array(poses)

    # set planning pipeline
    # Available planers ompl, chomp, stomp
    th.move_group.set_planning_pipeline_id("stomp")

    # set planner id, see {planner}_planning_pipeline.launch available options
    # th.move_group.set_planner_id('RRTConnect')

    # plan to named position,
    # see srdf and trajectory_handler for available options
    th.move_group.set_named_target("start")

    # plan to pose
    for pose in poses:
        th.move_group.set_pose_target(pose)
        success, plan = th.move_group.plan()[:2]
        if success:
            th.move_group.execute(plan, wait=True)
        else:
            return rospy.loginfo(f"{th.name}: planning failed, robot program aborted")

    # plan to joint target
    th.move_group.set_joint_value_target(
        th.create_goal((0.0, -pi / 2.0, pi / 2.0, 0.0, pi / 2.0, -pi / 2))
    )
    success, plan = th.move_group.plan()[:2]
    if success:
        th.move_group.execute(plan, wait=True)
    else:
        return rospy.loginfo(f"{th.name}: planning failed, robot program aborted")

    # create collision object
    co_pose = PoseStamped()
    co_pose.header = Header()
    co_pose.header.frame_id = th.move_group.get_planning_frame()
    co_pose.pose = Pose(position=Point(1, 0, 0.2), orientation=Quaternion(0, 0, 0, 1))

    co = CollisionObject()
    co.operation = CollisionObject.ADD
    co.id = "box"
    co.header = co_pose.header
    co.pose = co_pose.pose
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [1, 0.2, 0.5]
    co.primitives = [box]

    th.add_collision_object(co)

    for pose in poses:
        th.move_group.set_pose_target(pose)
        success, plan = th.move_group.plan()[:2]
        if success:
            th.move_group.execute(plan, wait=True)
        else:
            return rospy.loginfo(f"{th.name}: planning failed, robot program aborted")

    return rospy.loginfo(f"{th.name}: robot program completed")


if __name__ == "__main__":

    robot_program()
