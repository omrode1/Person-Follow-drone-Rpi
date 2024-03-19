import cv2
import numpy as np
from picamera2 import Picamera2

cams = []

def create_camera():
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    cams.append(camera)

def get_image_size(camera_id):
    return 640, 480  # Assuming fixed resolution for Pi Camera

def get_video(camera_id):
    camera = cams[camera_id]
    img = np.empty((480, 640, 3), dtype=np.uint8)
    camera.capture(img, format='bgr')
    return img

def close_cameras():
    for camera in cams:
        camera.close()

if __name__ == "__main__":
    create_camera()
    
    while True:
        img = get_video(0)
        cv2.imshow("camera", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    close_cameras()
