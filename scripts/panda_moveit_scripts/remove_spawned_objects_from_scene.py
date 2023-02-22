import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String


class MoveGroup(object):
    """MoveGroup"""

    def __init__(self):
        super(MoveGroup, self).__init__()

        # Initilializing moveit_commander and a rospy node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        # Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        # kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        # Instantiate a PlanningSceneInterface object.  This provides a remote interface
        # for getting, setting, and updating the robot's internal understanding of the surrounding world
        scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a MoveGroupCommander  object. This object is an interface
        # to a planning group (group of joints). The group is the primary
        # arm joints in the Panda robot, so we set the group's name to "panda_arm".
        # This interface can be used to plan and execute motions
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        # Create a DisplayTrajectory ROS publisher which is used to display trajectories in Rviz
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20
        )

        # Get the name of the reference frame for this robot
        planning_frame = move_group.get_planning_frame()
        print("\nPlanning frame: %s" % planning_frame)

        # Get the name of the link that is considered to be an end-effector. Will be empty string if no end effector
        eef_link = move_group.get_end_effector_link()
        print("End effector link: %s\n" % eef_link)

        # We can get a list of all the groups in the robot
        group_names = robot.get_group_names()
        print("Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the robot
        print("\nPrinting robot state: \n")
        print(robot.get_current_state())

        self.box_name = "box"
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        box_name = self.box_name
        scene = self.scene

        ## Ensuring Collision Updates Are Receieved
        ## call this function after adding, removing, attaching or detaching an object in
        ## the planning scene. We then wait until the updates have been made or
        ## ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

            # If we exited the while loop without returning then we timed out
        return False

    def remove_box(self, timeout=4):
        # Copy class variables to local variables
        box_name = self.box_name
        scene = self.scene

        ## We can remove the box from the world.
        scene.remove_world_object(box_name)

        ## **Note:** The object must be detached before we can remove it from the world

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)


def main():
    try:
        # Set up the moveit_commander
        environment = MoveGroup()
        print("Adding box to scene")
        environment.remove_box()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
