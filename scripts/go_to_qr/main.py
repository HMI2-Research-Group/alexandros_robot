from pyrealsense2 import pyrealsense2
import numpy as np
import cv2
import rospy
from pyzbar import pyzbar


def qr_code_data(image):
    barcodes = pyzbar.decode(image)
    for barcode in barcodes:
        # extract the bounding box location of the barcode and draw the
        # bounding box surrounding the barcode on the image
        (x, y, w, h) = barcode.rect
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
        # the barcode data is a bytes object so if we want to draw it on
        # our output image we need to convert it to a string first
        barcodeData = barcode.data.decode("utf-8")
        barcodeType = barcode.type
        # draw the barcode data and barcode type on the image
        text = "{} ({})".format(barcodeData, barcodeType)
        cv2.putText(image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        # print the barcode type and data to the terminal
        print("[INFO] Found {} barcode: {}".format(barcodeType, barcodeData))
        return barcodeData


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
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            # Detect QR Code
            # show the color image
            cv2.imshow("RealSense", color_image)
            qr_code_data(color_image)
            # cv2 waitkey
            key = cv2.waitKey(1)
            if key & 0xFF == ord("q") or key == 27:
                cv2.destroyAllWindows()
                break
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
