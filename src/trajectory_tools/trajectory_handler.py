#!/usr/bin/env python
from math import atan2, cos, pi, sin, sqrt
from sys import argv
from typing import List

import moveit_commander
import moveit_msgs.msg
import pyassimp
import rospy
import tf.transformations
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, Quaternion
from move_group_sequence.move_group_sequence import (
    Circ,
    Lin,
    MoveGroupSequence,
    Ptp,
    Sequence,
    from_euler,
)
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from sensor_msgs.msg import JointState
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from std_msgs.msg import Header
from ur_msgs.srv import SetIO
from visualization_msgs.msg import Marker, MarkerArray
from yaml import safe_load

PLANNING_GROUP = "manipulator"


class TrajectoryHandler:
    """Convenience class for interfacing with the Moveit Commander and planning and execution of trajectories."""
    def __init__(self, disable_io: bool = True) -> None:

        moveit_commander.roscpp_initialize(argv)
        rospy.init_node("trajectory_handler", anonymous=True)
        self.name = rospy.get_name()
        rospy.loginfo(f"{self.name}: node started")

        # Instantiate a `RobotCommander`_ object. Provides information such as
        # the robot's kinematic model and the robot's current joint states
        self.robot = moveit_commander.RobotCommander()

        # Instantiate a `PlanningSceneInterface`_ object.  This provides a
        # remote interface for getting, setting, and updating the robot's
        # internal understanding of the surrounding world
        self.scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a `MoveGroupCommander`_ object.  This object is an
        # interface to a planning group (group of joints).
        self.move_group = moveit_commander.MoveGroupCommander(PLANNING_GROUP)

        # Create a `DisplayTrajectory` ROS publisher which is used to display
        # move group sequence trajectories in Rviz
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # Instantiate a `MoveGroupSequence` for use of Pilz Industrial Motion
        # Planner
        self.sequencer = MoveGroupSequence(self.move_group)

        # Wait to ensure environment is ready
        rospy.sleep(0.2)

        # clear scene of objects and markers
        self.scene.clear()
        clear_marker_array()

        # Setup planning scene
        # Define robot workspace bounds
        # self.move_group.set_workspace((-1.5, -1.5, -0.2, 2, 1.5, 1.5))

        # Define named  joint targets
        self.set_named_targets()

        # Add static collision object to the scene
        self.setup_scene()

        if not disable_io:
            rospy.loginfo(f"{self.name}: waiting for /ur_hardware_interface/set_io")
            rospy.wait_for_service("ur_hardware_interface/set_io", 30)
            self.set_io = rospy.ServiceProxy("ur_hardware_interface/set_io", SetIO)

    def create_goal(self, joint_values: List[float]) -> JointState:

        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.header.frame_id = "base_link"
        joint_state.name = self.move_group.get_active_joints()
        joint_state.position = joint_values

        return joint_state

    def set_named_targets(self) -> None:

        self.start = (0.0, -pi / 2.0, pi / 2.0, 0.0, pi / 2.0, 0.0)
        self.move_group.remember_joint_values("start", list(self.start))
        self.start_ee = (0.0, -pi / 2.0, pi / 2.0, 0.0, pi / 2.0, -pi / 2.0)
        self.move_group.remember_joint_values("start_ee", list(self.start_ee))

    def display_trajectory(self) -> None:

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(self.sequencer._trajectories[0])

        self.display_trajectory_publisher.publish(display_trajectory)

    def add_collision_object(self, co: CollisionObject) -> bool:

        self.scene.add_object(co)

        return self.wait_for_state_update(f"{co.id}", object_is_known=True)

    def attach_end_effector(
        self,
        name: str,
        filename: str = "/dev_ws/src/trajectory_tools/end_effector/ur_ee.stl",
    ) -> bool:

        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = self.move_group.get_end_effector_link()
        pose.pose = Pose(position=Point(0, 0, 0), orientation=Quaternion(0, 0, 0, 1))

        ac = AttachedCollisionObject()
        ac.object = make_mesh(name, pose, filename=filename)

        ac.link_name = self.move_group.get_end_effector_link()
        ac.object.subframe_names = ["tcp"]
        ori = tf.transformations.quaternion_from_euler(0, pi / 2, 0)
        ac.object.subframe_poses = [
            Pose(
                position=Point(0.184935, 0.0, 0.06),
                orientation=Quaternion(ori[0], ori[1], ori[2], ori[3]),
            )
        ]

        self.scene.attach_object(ac)

        return self.wait_for_state_update(name, object_is_attached=True)

    def attach_camera(self, name: str) -> bool:

        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = self.move_group.get_end_effector_link()
        pose.pose = Pose(
            position=Point(0, 0, 0.0115), orientation=Quaternion(0, 0, 0, 1)
        )

        ac = AttachedCollisionObject()
        ac.object.id = name
        ac.object.header = pose.header
        ac.object.pose = pose.pose
        ac.object.operation = CollisionObject.ADD
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.042, 0.042, 0.023]
        ac.object.primitives = [box]
        ac.link_name = self.move_group.get_end_effector_link()
        ac.object.subframe_names = ["tcp"]
        ac.object.subframe_poses = [
            Pose(position=Point(0.0, 0.0, 0.001), orientation=Quaternion(0, 0, 0, 1))
        ]

        self.scene.attach_object(ac)

        return self.wait_for_state_update(name, object_is_attached=True)

    def setup_scene(self) -> bool:

        name = "table"
        pose = PoseStamped()
        pose.header.frame_id = self.robot.get_planning_frame()
        pose.header.stamp = rospy.Time.now()
        pose.pose = Pose(
            position=Point(0, 0, -0.51), orientation=Quaternion(0, 0, 0, 1)
        )
        self.scene.add_box(name, pose, (1, 1, 1))

        return self.wait_for_state_update(name, object_is_known=True)

    def wait_for_state_update(
        self,
        object_name: str,
        object_is_attached: bool = False,
        object_is_known: bool = False,
        timeout: int = 4,
    ) -> bool:

        start = rospy.get_time()
        seconds = rospy.get_time()

        while (seconds - start < timeout) and not rospy.is_shutdown():

            attached_objects = self.scene.get_attached_objects([object_name])
            is_attached = len(attached_objects.keys()) > 0

            is_known = object_name in self.scene.get_known_object_names()

            if (object_is_attached == is_attached) and (object_is_known == is_known):
                rospy.loginfo(f"{self.name}: {object_name} added to scene")
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()

        rospy.logerr(f"{self.name}: failed to add {object_name} to scene")

        return False

    @staticmethod
    def publish_marker_array(poses: List[Pose]) -> None:

        pub = rospy.Publisher("pose_marker_array", MarkerArray, queue_size=100)

        marker_array = MarkerArray()

        rotation_y = tf.transformations.quaternion_about_axis(-pi / 2, (0, 0, 1))
        rotation_z = tf.transformations.quaternion_about_axis(-pi / 2, (0, 1, 0))

        color_x = (1.0, 0.0, 0.0, 0.6)
        color_y = (0.0, 1.0, 0.0, 0.6)
        color_z = (0.0, 0.0, 1.0, 0.6)

        scale_x = (0.05, 0.005, 0.005)
        scale_y = (0.05, 0.005, 0.005)
        scale_z = (0.08, 0.005, 0.005)

        id = 0

        for pose in poses:

            y_ori = tf.transformations.quaternion_multiply(
                rotation_y,
                [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ],
            )

            z_ori = tf.transformations.quaternion_multiply(
                rotation_z,
                [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ],
            )

            y_axis = Pose(
                position=Point(pose.position.x, pose.position.y, pose.position.z),
                orientation=Quaternion(y_ori[0], y_ori[1], y_ori[2], y_ori[3]),
            )

            z_axis = Pose(
                position=Point(pose.position.x, pose.position.y, pose.position.z),
                orientation=Quaternion(z_ori[0], z_ori[1], z_ori[2], z_ori[3]),
            )

            for axis, color, scale in zip(
                [pose, y_axis, z_axis],
                [color_x, color_y, color_z],
                [scale_x, scale_y, scale_z],
            ):

                marker = Marker()
                marker.color.r = color[0]
                marker.color.g = color[1]
                marker.color.b = color[2]
                marker.color.a = color[3]

                marker.header = Header()
                marker.header.frame_id = "base_link"
                marker.header.stamp = rospy.Time.now()
                marker.id = id
                marker.scale.x = scale[0]
                marker.scale.y = scale[1]
                marker.scale.z = scale[2]
                marker.pose = axis
                marker.type = Marker.ARROW
                marker.action = Marker.ADD

                marker_array.markers.append(marker)

                id += 1

        for i in range(3):
            pub.publish(marker_array)
            rospy.sleep(0.2)


def make_mesh(
    name: str, pose: PoseStamped, filename: str, scale=(1, 1, 1)
) -> CollisionObject:

    co = CollisionObject()
    if pyassimp is False:
        rospy.logerr(
            "Pyassimp needs patch https://launchpadlibrarian.net/319496602/patchPyassim.txt"
        )
    scene = pyassimp.load(filename)
    if not scene.meshes or len(scene.meshes) == 0:
        rospy.logerr("There are no meshes in the file")
    if len(scene.meshes[0].faces) == 0:
        rospy.logerr("There are no faces in the mesh")
    co.operation = CollisionObject.ADD
    co.id = name
    co.header = pose.header
    co.pose = pose.pose

    mesh = Mesh()
    first_face = scene.meshes[0].faces[0]
    if hasattr(first_face, "__len__"):
        for face in scene.meshes[0].faces:
            if len(face) == 3:
                triangle = MeshTriangle()
                triangle.vertex_indices = [face[0], face[1], face[2]]
                mesh.triangles.append(triangle)
    elif hasattr(first_face, "indices"):
        for face in scene.meshes[0].faces:
            if len(face.indices) == 3:
                triangle = MeshTriangle()
                triangle.vertex_indices = [
                    face.indices[0],
                    face.indices[1],
                    face.indices[2],
                ]
                mesh.triangles.append(triangle)
    else:
        rospy.logerr("Unable to build triangles from mesh due to mesh object structure")
    for vertex in scene.meshes[0].vertices:
        point = Point()
        point.x = vertex[0] * scale[0]
        point.y = vertex[1] * scale[1]
        point.z = vertex[2] * scale[2]
        mesh.vertices.append(point)
    co.meshes = [mesh]
    pyassimp.release(scene)
    return co


def poses_from_yaml(filepath: str) -> List[Pose]:

    poses = []

    with open(filepath, "r", encoding="utf-8") as file:
        y = safe_load(file)
        for pose in y.get("path"):
            p = Pose()
            p.position.x = pose["position"][0]
            p.position.y = pose["position"][1]
            p.position.z = pose["position"][2]
            p.orientation.w = pose["quaternion"][0]
            p.orientation.x = pose["quaternion"][1]
            p.orientation.y = pose["quaternion"][2]
            p.orientation.z = pose["quaternion"][3]
            poses.append(p)

    return poses


def publish_poses_as_pose_array(poses: List[Pose]) -> None:

    pub = rospy.Publisher("pose_array_publisher", PoseArray, queue_size=10)

    pose_array = PoseArray()
    pose_array.header = Header()
    pose_array.header.frame_id = "base_link"
    pose_array.header.stamp = rospy.Time.now()
    pose_array.poses = poses

    for counter in range(10):
        pub.publish(pose_array)
        rospy.sleep(0.1)


def clear_marker_array() -> None:

    pub = rospy.Publisher("pose_marker_array", MarkerArray, queue_size=10)

    marker_array = MarkerArray()

    marker = Marker()
    marker.header = Header()
    marker.header.stamp = rospy.Time.now()
    marker.id = 0
    marker.action = Marker.DELETEALL

    marker_array.markers.append(marker)

    for i in range(3):
        pub.publish(marker_array)
        rospy.sleep(0.2)


# functions below from
# https://github.com/o2ac/o2ac-ur/blob/74c82a54a693bf6a3fc995ff63e7c91ac1fda6fd/catkin_ws/src/o2ac_routines/src/o2ac_routines/helpers.py#L356


def _norm2(a, b, c=0.0):
    return sqrt(a**2 + b**2 + c**2)


def ur_axis_angle_to_quat(axis_angle: List[float]) -> List[float]:
    # https://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation#Unit_quaternions
    angle = TrajectoryHandler._norm2(*axis_angle)
    axis_normed = [axis_angle[0] / angle, axis_angle[1] / angle, axis_angle[2] / angle]
    s = sin(angle / 2)
    return [
        s * axis_normed[0],
        s * axis_normed[1],
        s * axis_normed[2],
        cos(angle / 2),
    ]  # xyzw


def quat_to_ur_axis_angle(quaternion: List[float]) -> List[float]:
    # https://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation#Unit_quaternions
    # quaternion must be [xyzw]
    angle = 2 * atan2(
        TrajectoryHandler._norm2(quaternion[0], quaternion[1], quaternion[2]),
        quaternion[3],
    )
    if abs(angle) > 1e-6:
        axis_normed = [
            quaternion[0] / sin(angle / 2),
            quaternion[1] / sin(angle / 2),
            quaternion[2] / sin(angle / 2),
        ]
    else:
        axis_normed = 0.0
    return [axis_normed[0] * angle, axis_normed[1] * angle, axis_normed[2] * angle]
