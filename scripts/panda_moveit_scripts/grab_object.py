#! /usr/bin/env python3
import franka_gripper.msg
import rospy
import sys

# Brings in the SimpleActionClient
import actionlib


# Brings in the messages used by the grasp action, including the
# goal message and the result message.


def grasp_client():
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
    goal.epsilon.inner = obj_width / 2
    goal.epsilon.outer = obj_width / 2
    goal.speed = 0.08
    goal.force = 2.0

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A GraspResult


if __name__ == "__main__":
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node("grasp_client_py")
        result = grasp_client()
        print("Success: ", result.success)
        print("Error message: ", result.error)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
