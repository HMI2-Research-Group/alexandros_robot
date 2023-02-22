import rospy
from geometry_msgs.msg import PoseStamped


def main():
    rospy.init_node("ridgeback_set_goal")
    pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    rospy.sleep(1)
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.pose.position.x = -0.033
    goal.pose.position.y = 1.544
    goal.pose.position.z = 0.0
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = 0.462
    goal.pose.orientation.w = 0.886
    pub.publish(goal)


if __name__ == "__main__":
    main()
