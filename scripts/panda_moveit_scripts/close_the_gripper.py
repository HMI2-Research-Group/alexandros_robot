# This file contains API to close the gripper in Panda robot using moveit
from __future__ import print_function
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos


def close_gripper():
    ## First initialize moveit_commander and rospy.
    print("============ Starting tutorial setup")
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("close_gripper", anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "panda_arm"
    group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20
    )
    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print("============ Reference frame: %s" % planning_frame)
    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print("============ End effector: %s" % eef_link)
    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("============ Robot Groups:", robot.get_group_names())
    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state")
    print(robot.get_current_state())
    print("============")

    # We can get the joint values from the group and adjust some of the values:
    my_goal = [
        0.2750929313550271,
        0.4611823460472524,
        -0.4240702996798174,
        -1.661900412198573,
        0.21080957110387105,
        2.077615107127882,
        0.572222709304476,
    ]
    # my_goal = [0.01, 0.01]
    group.go(my_goal, wait=True)
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()
    # When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()
    # END_TUTORIAL


if __name__ == "__main__":
    try:
        close_gripper()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion", file=sys.stderr)
