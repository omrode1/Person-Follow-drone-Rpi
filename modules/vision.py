import cv2
import numpy as np
from picamera2 import Picamera2

camera = None

def initialize_detector():
    global camera
    camera = Picamera2()
    camera.resolution = (640, 480)
    camera.framerate = 32
    print("Camera initialized")

def get_image_size():
    return 640, 480  # Assuming fixed resolution for Pi Camera

def close_camera():
    global camera
    if camera:
        camera.close()
        print("Camera closed")

def get_detections():
    person_detections = []
    img = np.empty((480, 640, 3), dtype=np.uint8)
    camera.capture(img, format='bgr')
    
    # Assuming some method to get person_detections, you may need to replace this
    # For example, using OpenCV contour detection:
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    person_detections = contours

    return person_detections, img

def getCenter(contour):
    M = cv2.moments(contour)
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return cx, cy

def getDelta(point1, point2):
    return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

def process(output_img, person_detections):
    gray = cv2.cvtColor(output_img, cv2.COLOR_BGR2GRAY)

    filtered_contours = person_detections

    centerpoint = (round(output_img.shape[1] / 2), round(output_img.shape[0] / 2))

    count = 0
    target = ((centerpoint[0], centerpoint[1]), 0)
    if len(filtered_contours) > 1:
        for contour in filtered_contours:
            targetCenter = getCenter(contour)
            if count == 0:
                targetCenter = getCenter(contour)
                delta = getDelta(targetCenter, centerpoint)
                target = (targetCenter, delta)
            else:
                currentDelta = getDelta(targetCenter, centerpoint)
                if abs(target[1]) > abs(currentDelta):
                    targetCenter = getCenter(contour)
                    target = (targetCenter, currentDelta)
            count += 1

    elif len(filtered_contours) == 1:
        targetCenter = getCenter(filtered_contours[0])
        delta = getDelta(targetCenter, centerpoint)
        target = (targetCenter, delta)

    else:
        cv2.putText(output_img, "NO TARGET", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3, cv2.LINE_AA)

    # show target
    cv2.circle(output_img, target[0], 20, (0, 0, 255), thickness=-1, lineType=8, shift=0)
    cv2.putText(output_img, str(target[1]), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3, cv2.LINE_AA)
    # show path
    cv2.line(output_img, centerpoint, target[0], (255, 0, 0), thickness=10, lineType=8, shift=0)

    # show center
    cv2.circle(output_img, centerpoint, 20, (0, 255, 0), thickness=-1, lineType=8, shift=0)

    # show contours
    cv2.drawContours(output_img, filtered_contours, -1, (255, 255, 255), 3)

    return output_img

if __name__ == "__main__":
    try:
        initialize_detector()

        while True:
            detections, img = get_detections()

            # Process the detections as needed
            processed_img = process(img, detections)

            cv2.imshow("Detected Objects", processed_img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        close_camera()
        cv2.destroyAllWindows()
