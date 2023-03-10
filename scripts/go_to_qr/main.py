from pyrealsense2 import pyrealsense2
import numpy as np
import cv2
import rospy
from pyzbar import pyzbar
from geometry_msgs.msg import Twist

speed_robot = None
pub = None  # rospy.Publisher("/cmd_vel", Twist, queue_size=100)


def qr_code_data(interested_barcode, image, aligned_depth_image):
    barcodes = pyzbar.decode(image)
    for barcode in barcodes:
        # extract the bounding box location of the barcode and draw the
        # bounding box surrounding the barcode on the image
        (x, y, w, h) = barcode.rect
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
        # the barcode data is a bytes object so if we want to draw it on
        # our output image we need to convert it to a string first
        # Get the midpoint of the QR code
        x_mid = x + w / 2
        y_mid = y + h / 2
        depth = aligned_depth_image.get_distance(int(x_mid), int(y_mid))
        barcodeData = barcode.data.decode("utf-8")
        barcodeType = barcode.type
        # draw the barcode data and barcode type on the image
        text = f"X: {x_mid}, Y: {y_mid}, Depth: {depth}"
        cv2.putText(image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        # print the barcode type and data to the terminal
        # print("[INFO] Found {} barcode: {}".format(barcodeType, barcodeData))
        return (x_mid, y_mid, depth)
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


def go_left():
    move_robot(0.0, speed_robot, 0.0)


def go_right():
    move_robot(0.0, -speed_robot, 0.0)


def go_forward():
    move_robot(speed_robot, 0.0, 0.0)


def go_backward():
    move_robot(-speed_robot, 0.0, 0.0)


def rotate():
    move_robot(0.0, 0.0, speed_robot)


def command_robot(x_mid, y_mid, depth):
    # mid_threshold = 5
    # if depth > 0.7:
    #     print(f"Current depth: {depth}")
    #     go_forward()
    # else:
    #     return True
    pass


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
            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue
            color_image = np.asanyarray(color_frame.get_data())
            # Detect QR Code
            # show the color image
            qr_output_data = qr_code_data("Location 1", color_image, aligned_depth_frame)
            if qr_output_data is not None:
                reached_destination = command_robot(*qr_output_data)
                # if reached_destination:
                #     break
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
    speed_robot = 0.1
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
    main()
