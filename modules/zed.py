import cv2
import time
import numpy as np
import pyrealsense2 as rs
from picamera2 import PiCamera
from picamera2.array import PiRGBArray

# Realsense required variables
pipeline = None
colorizer = None

# PiCamera required variables
camera = None

MAX_RANGE = 6
MIN_RANGE = 1

def set_params():
    # No specific parameters for PiCamera
    pass

def close():
    global pipeline, colorizer, camera
    if pipeline:
        pipeline.stop()
    if camera:
        camera.close()

def init_realsense():
    global pipeline, colorizer

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    pipeline.start(config)

    colorizer = rs.colorizer()

def init_picamera():
    global camera

    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 30

def get_rgbd_image_realsense():
    global colorizer

    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

    if not depth_frame or not color_frame:
        return None

    colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())
    rgb_image = np.asanyarray(color_frame.get_data())

    bgrd = np.concatenate((rgb_image, colorized_depth), axis=1)
    return bgrd

def get_rgbd_image_picamera():
    global camera

    raw_capture = PiRGBArray(camera, size=(640, 480))
    camera.capture(raw_capture, format="bgr", use_video_port=True)
    rgb_image = raw_capture.array
    raw_capture.truncate(0)

    # Dummy depth image (all zeros) since PiCamera doesn't provide depth information
    depth_image = np.zeros_like(rgb_image[:, :, 0], dtype=np.uint16)

    bgrd = np.concatenate((rgb_image, cv2.cvtColor(depth_image, cv2.COLOR_GRAY2BGR)), axis=1)
    return bgrd

if __name__ == "__main__":
    use_realsense = False  # Set to True if using Intel RealSense, False for PiCamera

    if use_realsense:
        init_realsense()
        get_rgbd_image = get_rgbd_image_realsense
    else:
        init_picamera()
        get_rgbd_image = get_rgbd_image_picamera

    while True:
        bgrd = get_rgbd_image()
        cv2.imshow("BGRD", bgrd)
        cv2.waitKey(1)

    close()
