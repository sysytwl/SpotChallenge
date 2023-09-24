import numpy as np
import cv2
import cv2.aruco as aruco

from picamera2 import Picamera2, MappedArray, Preview

import time



# Initialize the camera and grab a reference to the raw camera capture
picam2 = Picamera2()
picam2.start_preview(Preview.NULL)
#picam2.start_preview(Preview.DRM)
config = picam2.create_preview_configuration({"size": (640, 480),"format": "BGR888"})
picam2.configure(config)

# Define the ArUco dictionary and parameters
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
aruco_params = aruco.DetectorParameters_create()

# Define the intrinsic parameters of the camera
camera_matrix = np.array([[573.7, 0, 360.4],
                          [0, 592.0, 205.9],
                          [0,     0,     1]])

# Define the distortion coefficients of the camera
dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

# OSD
color0 = (0, 255, 0)
color1 = (255, 0, 0)
font = cv2.FONT_HERSHEY_SIMPLEX
scale = 0.5
thickness = 1

# record
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec for MP4 format
output_file = 'output_video.mp4'  # Output video file name
frame_size = (640, 480)  # Specify the frame size
out = cv2.VideoWriter(output_file, fourcc, 20.0, frame_size)
prev_time = time.time()
fps_update_interval = 5  # Update fps every 5 seconds


def aruco_marker(request):
    with MappedArray(request, "main") as m:
        gray = cv2.cvtColor(m.array, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(m.array, aruco_dict, parameters=aruco_params)
        if ids is not None:
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 20, camera_matrix, dist_coeffs)
            print(tvec)

            aruco.drawDetectedMarkers(m.array, corners, ids)
            for i in range(len(ids)):
                aruco.drawAxis(m.array, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1)

            cv2.putText(m.array, str(rvecs[0]), (30, 60), font, scale, color1, thickness)
            cv2.putText(m.array, str(tvecs[0]), (30, 90), font, scale, color1, thickness)

        current_time = time.time()
        elapsed_time = current_time - prev_time
        cv2.putText(m.array, str(current_fps), (30, 30), font, scale, color1, thickness)

        out.write(m.array)

        if elapsed_time > fps_update_interval:
            current_fps = int(1 / elapsed_time)
            out.set(cv2.CAP_PROP_FPS, current_fps)
            prev_time = current_time



if __name__ == '__main__':
    picam2.pre_callback = aruco_marker
    picam2.start()
    while cv2.waitKey(1) & 0xFF != 27: #ESC
        time.sleep(1)
    picam2.stop()
