import rospy
from geometry_msgs.msg import PoseStamped


def main():
    rospy.init_node("ridgeback_set_goal")
    pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    rospy.sleep(1)
    goal = PoseStamped()
    goal.header.frame_id = "odom"
    goal.pose.position.x = -0.4
    goal.pose.position.y = 3.9
    goal.pose.position.z = 0.0
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = -0.619
    goal.pose.orientation.w = 0.786
    pub.publish(goal)


if __name__ == "__main__":
    main()
