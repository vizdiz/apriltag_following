from threading import Thread, Event
from time import sleep

from pid import PID
from video import Video
from bluerov_interface import BlueROV
from pymavlink import mavutil
from dt_apriltags import Detector

# TODO: import your processing functions

from apriltag_processing import *

# Create the video object
video = Video()
# Create the PID object
pid_vertical = PID(K_p=0.1, K_i=0.0, K_d=0.01, integral_limit=1)
pid_horizontal = PID(K_p=0.1, K_i=0.0, K_d=0.01, integral_limit=1)
# Create the mavlink connection
mav_comn = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
# Create the BlueROV object
bluerov = BlueROV(mav_connection=mav_comn)

frame = None
frame_available = Event()
frame_available.set()

vertical_power = 0
lateral_power = 0


def _get_frame():
    global frame
    while not video.frame_available():
        print("Waiting for frame...")
        sleep(0.01)

    at_detector = Detector(
        families="tag36h11",
        nthreads=1,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0,
    )

    try:
        while True:
            if video.frame_available():
                frame = video.frame()

                # TODO: Add frame processing here
                try:
                    height, width, channels = frame.shape

                    tags = detect_april_tags(frame, at_detector)

                    x_raw_error, y_raw_error = determine_error(tags, (width, height))

                    errors = (x_raw_error, y_raw_error)
                    pid_controllers = (pid_horizontal, pid_vertical)

                    x_output, y_output = calculate_pid_output(errors, pid_controllers)

                    # TODO: set vertical_power and lateral_power here
                    lateral_power = x_output
                    vertical_power = y_output

                except Exception as e:
                    print(e)

                print(frame.shape)

    except KeyboardInterrupt:
        return


def _send_rc():
    global lateral_power, vertical_power

    bluerov.set_vertical_power(vertical_power)
    bluerov.set_lateral_power(lateral_power)


# Start the video thread
video_thread = Thread(target=_get_frame)
video_thread.start()

# Start the RC thread
rc_thread = Thread(target=_send_rc)
rc_thread.start()

# Main loop
try:
    while True:
        mav_comn.wait_heartbeat()
except KeyboardInterrupt:
    video_thread.join()
    rc_thread.join()
    bluerov.disarm()
    print("Exiting...")
