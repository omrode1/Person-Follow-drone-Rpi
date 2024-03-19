import sys
import time
import argparse
import cv2
import collections
import keyboard
from picamera2 import Picamera2

import lidar
import detector_mobilenet as detector
import vision
import control

# Args parser
parser = argparse.ArgumentParser(description='Drive autonomous')
parser.add_argument('--debug_path', type=str, default="debug/run1", help='debug message name')
parser.add_argument('--mode', type=str, default='flight', help='Switches between flight record and flight visualisation')
parser.add_argument('--control', type=str, default='PID', help='Use PID or P controller')
args = parser.parse_args()

# config
MAX_FOLLOW_DIST = 2                          # meter
MAX_ALT =  2.5                                # m
MAX_MA_X_LEN = 5
MAX_MA_Z_LEN = 5
MA_X = collections.deque(maxlen=MAX_MA_X_LEN)   # Moving Average X
MA_Z = collections.deque(maxlen=MAX_MA_Z_LEN)   # Moving Average Z
STATE = "takeoff"                               # takeoff land track search
# end config

def setup():
    print("connecting lidar")
    lidar.connect_lidar("/dev/ttyTHS1")

    print("setting up detector")
    detector.initialize_detector()

    print("connecting to drone")
    if args.mode == "flight":
        print("MODE = flight")
        control.connect_drone('/dev/ttyACM0')
    else:
        print("MODE = test")
        control.connect_drone('127.0.0.1:14551')

    control.set_flight_altitude(MAX_ALT)  # new never tested!

setup()

# initialize PiCamera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32

# initialize video writer
image_width, image_height = camera.resolution
debug_image_writer = cv2.VideoWriter(args.debug_path + ".avi", cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 25.0, (image_width, image_height))

control.configure_PID(args.control)
control.initialize_debug_logs(args.debug_path)

def track():
    print("State is TRACKING -> " + STATE)
    while True:
        if keyboard.is_pressed('q'):  # if key 'q' is pressed
            print("Closing due to manual interruption")
            land()  # Closes the loop and program

        img = camera.capture(format='bgr', use_video_port=True)

        # detections, fps, image = detector.get_detections()  # you need to implement this function

        # if len(detections) > 0:
            # your tracking logic here

def search():
    # implementation of search function
    pass

def takeoff():
    # implementation of takeoff function
    pass

def land():
    # implementation of land function
    pass

def visualize(img):
    # implementation of visualize function
    pass

def prepare_visualisation(lidar_distance, person_center, person_to_track, image, yaw_command, x_delta, y_delta, fps,
                          velocity_x_command, lidar_on_target):
    # implementation of prepare_visualisation function
    pass

def calculate_ma(ma_array):
    # implementation of calculate_ma function
    pass

while True:
    # main program loop
    """ True or False values depend on whether or not
        a PID controller or a P controller will be used  """

    if STATE == "track":
        control.set_system_state("track")
        STATE = track()

    elif STATE == "search":
        control.set_system_state("search")
        STATE = search()

    elif STATE == "takeoff":
        STATE = takeoff()

    elif STATE == "land":
        STATE = land()
