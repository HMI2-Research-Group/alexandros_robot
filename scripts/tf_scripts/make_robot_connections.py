import tf
import rospy


def main():
    # publish a tf frame connecting two frames
    br = tf.TransformBroadcaster()
    rospy.init_node("make_robot_connections")
    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        br.sendTransform(
            (0.25, 0.0, 0.405),
            tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time.now(),
            "panda_link0",
            "base_link",
        )
        br.sendTransform(
            (0.0, 0.0, 0.0),
            tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time.now(),
            "camera_link",
            "panda_link8",
        )
        rate.sleep()


if __name__ == "__main__":
    main()
