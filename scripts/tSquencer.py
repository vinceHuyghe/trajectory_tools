from move_group_sequence import MoveGroupSequence, Ptp, Lin, Circ, Sequence
from geometry_msgs.msg import Point, Quaternion, Pose
from moveit_commander import MoveGroupCommander
import rospy


def main():
    rospy.init_node('robot_program', anonymous=True)
    PLANNING_GROUP = "manipulator"
    move_group = MoveGroupCommander(PLANNING_GROUP)
    sequencer = MoveGroupSequence(move_group)
    
    home = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    
    # set start state
    sequencer.plan(Ptp(goal=home))
    sequencer.execute()

    sequence = Sequence()

    sequence.append(Ptp(goal=home))
    sequence.append(Ptp(goal=Pose(
        position=Point(0.3, 0.0, 0.5), 
        orientation=Quaternion(0.0, 1.0, 0.0, 0.0)
        )))
    sequence.append(Lin(goal=Pose(
        position=Point(0.3, 0.0, 0.3), 
        orientation=Quaternion(0.0, 1.0, 0.0, 0.0)
        ), vel_scale=0.1, acc_scale=0.05), blend_radius=0.01)
    sequence.append(Lin(goal=Pose(
        position=Point(0.5, 0.0, 0.3), 
        orientation=Quaternion(0.0, 1.0, 0.0, 0.0)
        ), vel_scale=0.1, acc_scale=0.05), blend_radius=0.01)
    sequence.append(Lin(goal=Pose(
        position=Point(0.1, 0.5, 0.3), 
        orientation=Quaternion(0.0, 1.0, 0.0, 0.0)
        )))

    sequencer.plan(sequence)

    sequencer.execute()

if __name__ == '__main__':
    main()
