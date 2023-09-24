import numpy as np
import cv2
import cv2.aruco as aruco
from picamera2 import Picamera2, Preview
import time

class ArucoMarkerDetector:
    def __init__(self):
        # Initialize the PiCamera
        self.picam2 = Picamera2()
        try:
            self.picam2.start_preview(Preview.DRM)
        except Exception as e:
            print(e)
            self.picam2.start_preview(Preview.NULL)
        config = self.picam2.create_preview_configuration({"size": (640, 480), "format": "BGR888"})
        self.picam2.configure(config)

        # ArUco dictionary and parameters
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters_create()

        # Intrinsic camera parameters
        self.camera_matrix = np.array([[573.7, 0, 360.4],
                                       [0, 592.0, 205.9],
                                       [0, 0, 1]])

        # Distortion coefficients
        self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

        # OSD (On-Screen Display) settings
        self.color0 = (0, 255, 0)
        self.color1 = (255, 0, 0)
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.scale = 0.5
        self.thickness = 1

        # Video recording settings
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.output_file = 'output_video.mp4'
        self.frame_size = (640, 480)
        self.out = cv2.VideoWriter(self.output_file, self.fourcc, 30.0, self.frame_size, isColor=False)

    def detect_marker(self):
        # Capture image from the camera
        array = self.picam2.capture_array()
        gray = cv2.cvtColor(array, cv2.COLOR_BGR2GRAY)

        # fix the error: UnboundLocalError
        tvecs = None

        # Detect ArUco markers
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 20, self.camera_matrix, self.dist_coeffs)

            # Draw detected markers and axes
            aruco.drawDetectedMarkers(gray, corners, ids)
            for i in range(len(ids)):
                aruco.drawAxis(gray, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.1)

            # Overlay rotation and translation vectors on the image
            cv2.putText(gray, str(rvecs[0]), (30, 60), self.font, self.scale, self.color1, self.thickness)
            cv2.putText(gray, str(tvecs[0]), (30, 90), self.font, self.scale, self.color1, self.thickness)

        # Write the frame to the output video
        self.out.write(gray)
        return tvecs

if __name__ == '__main__':         # Start capturing images from the camera
    marker_detector = ArucoMarkerDetector()
    marker_detector.picam2.start()
    try:
        while True:
            _tvecs = marker_detector.detect_marker()
            if _tvecs == None:
                pass
            else:
                print(_tvecs)
    except KeyboardInterrupt:
        pass
    marker_detector.out.release()
    marker_detector.picam2.stop()
