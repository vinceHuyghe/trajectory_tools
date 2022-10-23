#!/usr/bin/env python3

from math import pi

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from industrial_reconstruction_msgs.msg import NormalFilterParams
from industrial_reconstruction_msgs.srv import (StartReconstruction,
                                                StartReconstructionRequest,
                                                StopReconstruction,
                                                StopReconstructionRequest)
from move_group_sequence.move_group_sequence import (Circ, Lin, Ptp, Sequence,
                                                     from_euler)
from trajectory_tools.trajectory_handler import TrajectoryHandler

# reconstruction parameters
start_srv_req = StartReconstructionRequest()
start_srv_req.tracking_frame = "tool0"
start_srv_req.relative_frame = "base_link"
start_srv_req.translation_distance = 0.0
start_srv_req.rotational_distance = 0.0
start_srv_req.live = True
start_srv_req.tsdf_params.voxel_length = 0.02
start_srv_req.tsdf_params.sdf_trunc = 0.04
start_srv_req.tsdf_params.min_box_values = Vector3(x=0.0, y=0.0, z=0.0)
start_srv_req.tsdf_params.max_box_values = Vector3(x=0.0, y=0.0, z=0.0)
start_srv_req.rgbd_params.depth_scale = 1000
start_srv_req.rgbd_params.depth_trunc = 0.5
start_srv_req.rgbd_params.convert_rgb_to_intensity = False

stop_srv_req = StopReconstructionRequest()
# stop_srv_req.archive_directory = '/dev_ws/src.reconstruction/'
stop_srv_req.mesh_filepath = "/home/v/test.ply"
# stop_srv_req.normal_filters = [NormalFilterParams(
#                     normal_direction=Vector3(x=0.0, y=0.0, z=1.0), angle=90)]
# stop_srv_req.min_num_faces = 1000


def robot_program():

    ee_name = "D405"

    rospy.wait_for_service("/start_reconstruction")
    rospy.loginfo("robot program: waiting for /start_reconstruction srv")
    start_recon = rospy.ServiceProxy("/start_reconstruction", StartReconstruction)
    stop_recon = rospy.ServiceProxy("/stop_reconstruction", StopReconstruction)

    start = (0.0, -pi / 2.0, pi / 2.0, 0.0, pi / 2.0, 0.0)
    pose1 = Pose(
        position=Point(0.6, -0.5, 0.3), orientation=Quaternion(0.0, 1.0, 0.0, 0.0)
    )
    pose2 = Pose(
        position=Point(0.6, 0.5, 0.3), orientation=Quaternion(0.0, 1.0, 0.0, 0.0)
    )

    th = TrajectoryHandler()
    th.publish_marker_array([pose1, pose2])

    # attach camera and set new tcp
    th.attach_camera(ee_name)
    th.move_group.set_end_effector_link(f"{ee_name}/tcp")
    rospy.loginfo(
        f"{th.name}: end effector link set to {th.move_group.get_end_effector_link()}"
    )

    # Move into position to start reconstruction
    th.sequencer.plan(Ptp(goal=start, vel_scale=0.3, acc_scale=0.3))
    th.sequencer.execute()
    th.sequencer.plan(Ptp(goal=pose1, vel_scale=0.3, acc_scale=0.3))
    th.sequencer.execute()

    # Start reconstruction with service srv_req
    resp = start_recon(start_srv_req)

    if resp:
        rospy.loginfo("robot program: reconstruction started successfully")
    else:
        rospy.loginfo("robot program: failed to start reconstruction")

    th.sequencer.plan(Lin(goal=pose2, vel_scale=0.1, acc_scale=0.3))
    th.sequencer.execute()

    # Stop reconstruction with service srv_req
    resp = stop_recon(stop_srv_req)

    if resp:
        rospy.loginfo("robot program: reconstruction stopped successfully")
    else:
        rospy.loginfo("robot program: failed to stop reconstruction")


if __name__ == "__main__":

    robot_program()
