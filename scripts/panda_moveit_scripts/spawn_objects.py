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

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def add_box(self, xpos=0.0, ypos=0.0, zpos=0.0):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"

        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = xpos
        box_pose.pose.position.y = ypos
        box_pose.pose.position.z = zpos

        box_name = "box"
        self.scene.add_mesh(
            box_name, box_pose, "/home/student/PycharmProjects/panda_ws/src/alexandros_robot/meshes/Pasta_Box.obj"
        )

        # return self.wait_for_state_update(box_is_known=True, timeout=timeout)


def main():
    try:
        # Set up the moveit_commander
        environment = MoveGroup()

        print("Adding box to scene")
        environment.add_box(xpos=0.0, ypos=0.0, zpos=0.0)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
