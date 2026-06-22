import cv2
import numpy as np
from config import (
    ARUCO_DICT,
    SIZE_OF_MARKER,
    CAMERA_MATRIX,
    DIST_COEFFS
)

class ArUcoDetector:
    def __init__(self):
        self.dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        self.params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dict, self.params)

        s = SIZE_OF_MARKER
        self.marker_points = np.array([
            ( s/2, -s/2, 0),
            (-s/2, -s/2, 0),
            (-s/2,  s/2, 0),
            ( s/2,  s/2, 0)
        ], dtype=np.float64)

        self.camera_matrix = CAMERA_MATRIX
        self.dist_coeffs = DIST_COEFFS

    def process_frame(self, frame):
        if frame is None:
            return frame, None, None, None, None

        corners, ids, _ = self.detector.detectMarkers(frame)
        coords = None
        x_center = y_center = None

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            c = corners[0][0]
            x_center = int(np.mean([p[0] for p in c]))
            y_center = int(np.mean([p[1] for p in c]))
            cv2.circle(frame, (x_center, y_center), 5, (0, 0, 255), -1)

            success, rvecs, tvecs = cv2.solvePnP(
                self.marker_points,
                corners[0],
                self.camera_matrix,
                self.dist_coeffs
            )
            if success:
                coords = [tvecs[0][0], tvecs[1][0], tvecs[2][0]]

        return frame, coords, x_center, y_center, ids
