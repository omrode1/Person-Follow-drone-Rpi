import cv2
import numpy as np
from picamera import PiCamera
from picamera.array import PiRGBArray

net = None
camera = None

def initialize_detector():
    global net, camera
    net = jetson.inference.detectNet("ssd-mobilenet-v2")
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    print("fakka")

def get_image_size():
    return 640, 480  # Assuming fixed resolution for Pi Camera

def close_camera():
    camera.close()

def get_detections():
    person_detections = []
    raw_capture = PiRGBArray(camera, size=(640, 480))
    camera.capture(raw_capture, format="bgr", use_video_port=True)
    img = raw_capture.array
    raw_capture.truncate(0)

    detections = net.Detect(jetson.utils.cudaFromNumpy(img))
    for detection in detections:
        if detection.ClassID == 1:  # Remove unwanted classes
            person_detections.append(detection)
    
    fps = net.GetNetworkFPS()

    return person_detections, fps, img

if __name__ == "__main__":
    initialize_detector()

    while True:
        detections, fps, img = get_detections()
        
        # Process the detections as needed
        for detection in detections:
            print(f"Class: {detection.ClassID}, Confidence: {detection.Confidence}")

        cv2.imshow("Detected Objects", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    close_camera()
    cv2.destroyAllWindows()
