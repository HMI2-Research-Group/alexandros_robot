# This file contains API to close the gripper in Panda robot using moveit
from __future__ import print_function
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos


def go_to_pose(my_goal):
    ## First initialize moveit_commander and rospy.
    print("============ Starting tutorial setup")
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("close_gripper", anonymous=True)
    robot = moveit_commander.RobotCommander()
    group_name = "panda_arm"
    group = moveit_commander.MoveGroupCommander(group_name)
    # Get current joint values
    print("============ Printing robot joint values")
    print(group.get_current_joint_values())
    print("============")
    # We can get the joint values from the group and adjust some of the values:
    group.go(my_goal, wait=True)
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()
    # When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()
