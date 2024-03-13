import cv2
import numpy as np
from picamera2 import PiCamera
from picamera2.array import PiRGBArray

cams = []

def create_camera():
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))
    cams.append((camera, rawCapture))

def get_image_size(camera_id):
    return 640, 480  # Assuming fixed resolution for Pi Camera

def get_video(camera_id):
    camera, rawCapture = cams[camera_id]
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        img = frame.array
        rawCapture.truncate(0)
        return img

def close_cameras():
    for camera, _ in cams:
        camera.close()

if __name__ == "__main__":
    create_camera()
    
    while True:
        img = get_video(0)
        cv2.imshow("camera", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    close_cameras()
