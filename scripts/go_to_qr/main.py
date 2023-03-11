from pyrealsense2 import pyrealsense2
import numpy as np
import cv2
import rospy
from pyzbar import pyzbar
from geometry_msgs.msg import Twist
from math import atan2, pi

speed_robot = None
pub = None  # rospy.Publisher("/cmd_vel", Twist, queue_size=100)


def get_angle(qrcode, aligned_depth_image, depth_intrin):
    poly = qrcode.polygon
    x1, y1 = poly[0].x, poly[0].y
    x2, y2 = poly[1].x, poly[1].y
    x3, y3 = poly[2].x, poly[2].y
    # get 3d point
    depth_point1 = pyrealsense2.rs2_deproject_pixel_to_point(
        depth_intrin, [x1, y1], aligned_depth_image.get_distance(x1, y1)
    )
    depth_point2 = pyrealsense2.rs2_deproject_pixel_to_point(
        depth_intrin, [x2, y2], aligned_depth_image.get_distance(x2, y2)
    )
    depth_point3 = pyrealsense2.rs2_deproject_pixel_to_point(
        depth_intrin, [x3, y3], aligned_depth_image.get_distance(x3, y3)
    )
    # Get unit vector perpendicular to the plane containing depth_point1, depth_point2, depth_point3
    v1 = np.array(
        [depth_point1[0] - depth_point2[0], depth_point1[1] - depth_point2[1], depth_point1[2] - depth_point2[2]]
    )
    v2 = np.array(
        [depth_point1[0] - depth_point3[0], depth_point1[1] - depth_point3[1], depth_point1[2] - depth_point3[2]]
    )
    normal = np.cross(v1, v2)
    # Get unit vector pointing towards the camera
    camera_vector = np.array([1, 0, 0])
    # Get angle between the two vectors
    angle = atan2(np.linalg.norm(np.cross(camera_vector, normal)), np.dot(camera_vector, normal))
    # Convert to degrees
    angle = angle * 180 / pi
    return angle


def qr_code_data(interested_barcode, image, aligned_depth_image, depth_intrin):
    barcodes = pyzbar.decode(image)
    for barcode in barcodes:
        # extract the bounding box location of the barcode and draw the
        # bounding box surrounding the barcode on the image
        (x, y, w, h) = barcode.rect
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
        # the barcode data is a bytes object so if we want to draw it on
        # our output image we need to convert it to a string first
        # Get the midpoint of the QR code
        x_mid = int(x + w / 2)
        y_mid = int(y + h / 2)
        depth = aligned_depth_image.get_distance(x_mid, y_mid)
        # get 3d point
        depth_point = pyrealsense2.rs2_deproject_pixel_to_point(depth_intrin, [x_mid, y_mid], depth)
        # print(f"X:{depth_point[0]}, Y:{depth_point[1]}, Z:{depth_point[2]}")
        barcodeData = barcode.data.decode("utf-8")
        barcodeType = barcode.type
        print(f"Barcode Angle: {get_angle(barcode, aligned_depth_image, depth_intrin)}")
        # get 2 digits after the decimal point of the depth
        depth = float("{:.2f}".format(depth))
        # draw the barcode data and barcode type on the image
        # text = f"X: {x_mid}, Y: {y_mid}, \nDepth: {depth}"
        text = f"X: {x_mid}, Y: {y_mid}, \nDepth: {depth}"
        cv2.putText(image, text, (0, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        # print the barcode type and data to the terminal
        # print("[INFO] Found {} barcode: {}".format(barcodeType, barcodeData))
        # Change it to Ridgeback coordinates
        return (depth_point[2], -depth_point[0], -depth_point[1])
    return None


def move_robot(x_vel, y_vel, z_vel):
    start_time = rospy.Time.now()
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=100)
    while (rospy.Time.now() - start_time).to_sec() < 0.5:
        twist = Twist()
        twist.linear.x = x_vel
        twist.linear.y = y_vel
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = z_vel
        pub.publish(twist)


def command_robot(x, y, z):
    x_vel = 0.0
    y_vel = 0.0
    z_vel = 0.0
    if y > 0.1:
        y_vel = speed_robot
    elif y < -0.1:
        y_vel = -speed_robot
    else:
        y_vel = 0.0
    if x > 0.75:
        x_vel = speed_robot
    elif x < 0.7:
        x_vel = -speed_robot
    else:
        x_vel = 0.0
    if x_vel == 0.0 and y_vel == 0.0:
        return True
    move_robot(x_vel, y_vel, z_vel)


def main():
    pipeline = pyrealsense2.pipeline()
    # Create a config and configure the pipeline to stream
    # different resolutions of color and depth streams
    config = pyrealsense2.config()
    config.enable_stream(pyrealsense2.stream.depth, 640, 480, pyrealsense2.format.z16, 30)
    config.enable_stream(pyrealsense2.stream.color, 640, 480, pyrealsense2.format.bgr8, 30)
    # Align color and depth streams
    align_to = pyrealsense2.stream.color
    align = pyrealsense2.align(align_to)
    pipeline.start(config)
    try:
        while True:
            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()
            # Align the depth frame to color frame
            aligned_frames = align.process(frames)
            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
            color_frame = aligned_frames.get_color_frame()
            # get intrinsics
            depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue
            color_image = np.asanyarray(color_frame.get_data())
            # Detect QR Code
            # show the color image
            qr_output_data = qr_code_data("Location 1", color_image, aligned_depth_frame, depth_intrin)
            if qr_output_data is not None:
                reached_destination = command_robot(*qr_output_data)
                if reached_destination:
                    break
            cv2.imshow("RealSense", color_image)
            # cv2 waitkey
            key = cv2.waitKey(1)
            if key & 0xFF == ord("q") or key == 27:
                cv2.destroyAllWindows()
                break
    except ZeroDivisionError:
        print("error")


if __name__ == "__main__":
    rospy.init_node("go_to_qr", anonymous=True)
    speed_robot = 0.05
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
    main()
