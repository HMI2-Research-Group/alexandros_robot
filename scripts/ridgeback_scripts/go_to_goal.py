import rospy
from geometry_msgs.msg import Twist


def main():
    rospy.init_node("go_to_goal")
    t = 10.0
    vel = 0.1
    start_time = rospy.Time.now()
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=100)
    while (rospy.Time.now() - start_time).to_sec() < 1:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
    while (rospy.Time.now() - start_time).to_sec() < t:
        twist = Twist()
        twist.linear.x = vel
        twist.linear.y = vel
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)


if __name__ == "__main__":
    main()
