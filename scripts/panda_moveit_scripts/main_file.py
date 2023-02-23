# This file contains API to close the gripper in Panda robot using moveit
from __future__ import print_function
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos
import franka_gripper.msg
import rospy
import sys
from time import sleep

# Brings in the SimpleActionClient
import actionlib


class pick_and_place:
    def __init__(self):
        ## First initialize moveit_commander and rospy.
        print("============ Starting tutorial setup")
        moveit_commander.roscpp_initialize(sys.argv)

    def go_to_pose(self, my_goal):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("close_gripper", anonymous=True)
        moveit_commander.RobotCommander()
        group_name = "panda_arm"
        self.group = moveit_commander.MoveGroupCommander(group_name)
        # Get current joint values
        print("============ Printing robot joint values")
        print(self.group.get_current_joint_values())
        print("============")
        # We can get the joint values from the group and adjust some of the values:
        self.group.go(my_goal, wait=True)
        self.group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.group.clear_pose_targets()
        # When finished shut down moveit_commander.
        moveit_commander.roscpp_shutdown()

    def grasp_object(self):
        # Creates the SimpleActionClient, passing the type of the action
        # (GraspAction) to the constructor.
        client = actionlib.SimpleActionClient("/franka_gripper/grasp", franka_gripper.msg.GraspAction)

        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()

        # Creates a goal to send to the action server.
        obj_width = 2.76e-3
        goal = franka_gripper.msg.GraspGoal()
        goal.width = obj_width
        goal.epsilon.inner = obj_width / 3
        goal.epsilon.outer = obj_width / 3
        goal.speed = 0.08
        goal.force = 5.0

        # Sends the goal to the action server.
        client.send_goal(goal)

        # Waits for the server to finish performing the action.
        client.wait_for_result()

        # Prints out the result of executing the action
        return client.get_result()  # A GraspResult

    def open_gripper(self):
        # Creates the SimpleActionClient, passing the type of the action
        # (GraspAction) to the constructor.
        client = actionlib.SimpleActionClient("/franka_gripper/move", franka_gripper.msg.MoveAction)

        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()

        # Creates a goal to send to the action server.
        goal = franka_gripper.msg.GraspGoal()
        goal.width = 0.03
        goal.epsilon.inner = 0.04
        goal.epsilon.outer = 0.04
        goal.speed = 0.03
        goal.force = 2.0

        # Sends the goal to the action server.
        client.send_goal(goal)

        # Waits for the server to finish performing the action.
        client.wait_for_result()

        # Prints out the result of executing the action
        print(client.get_result())


if __name__ == "__main__":
    saleh = pick_and_place()
    saleh.go_to_pose(
        [
            -0.0423873459032753,
            0.05782741451149175,
            0.27509045853521996,
            -1.604960432372147,
            -0.004627947911206219,
            1.6792911282636798,
            1.0570489505046525,
        ]
    )
    saleh.grasp_object()
    sleep(5)
    saleh.go_to_pose(
        [
            -0.08905661694923449,
            -0.10107079806000167,
            0.27850159359769966,
            -1.6273816068716214,
            -0.012933404083881108,
            1.6975916814111935,
            1.0572583099421455,
        ]
    )
    sleep(5)
    saleh.go_to_pose(
        [
            -0.9768937012676092,
            -0.8773749955299577,
            2.260088546284458,
            -0.9371075568694284,
            0.6976467304693328,
            1.6223954304059345,
            0.7838235893651015,
        ]
    )
    sleep(5)
    saleh.open_gripper()
