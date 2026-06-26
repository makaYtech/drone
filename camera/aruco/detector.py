import cv2
import numpy as np
from config import SIZE_OF_MARKER, CAMERA_MATRIX, DIST_COEFFS

class ArUcoDetector:
    def __init__(self, dict_4x4=cv2.aruco.DICT_4X4_250, dict_5x5=cv2.aruco.DICT_5X5_250):
        self.dict_4x4 = cv2.aruco.getPredefinedDictionary(dict_4x4)
        self.dict_5x5 = cv2.aruco.getPredefinedDictionary(dict_5x5)
        self.params = cv2.aruco.DetectorParameters()
        self.detector_4x4 = cv2.aruco.ArucoDetector(self.dict_4x4, self.params)
        self.detector_5x5 = cv2.aruco.ArucoDetector(self.dict_5x5, self.params)

        s = SIZE_OF_MARKER
        self.marker_points = np.array([
            ( s/2, -s/2, 0),
            (-s/2, -s/2, 0),
            (-s/2,  s/2, 0),
            ( s/2,  s/2, 0)
        ], dtype=np.float64)
        self.camera_matrix = CAMERA_MATRIX
        self.dist_coeffs = DIST_COEFFS

    def _compute_marker_info(self, corners, marker_id, marker_type):
        """Вычисляет центр и 3D-координаты маркера."""
        corner = corners[0]
        x_center = int(np.mean([p[0] for p in corner]))
        y_center = int(np.mean([p[1] for p in corner]))
        success, rvecs, tvecs = cv2.solvePnP(
            self.marker_points, corners,
            self.camera_matrix, self.dist_coeffs
        )
        coords = [tvecs[0][0], tvecs[1][0], tvecs[2][0]] if success else None
        return {
            'id': int(marker_id),
            'type': marker_type,
            'coords': coords,
            'corners': corners,
            'center': (x_center, y_center)
        }

    def process_frame(self, frame):
        if frame is None:
            return frame, []

        markers = []

        # Детекция 4x4
        corners_4, ids_4, _ = self.detector_4x4.detectMarkers(frame)
        if ids_4 is not None and len(ids_4) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners_4, ids_4)
            for corner, marker_id in zip(corners_4, ids_4.flatten()):
                info = self._compute_marker_info(corner, marker_id, '4x4')
                markers.append(info)
                cv2.circle(frame, info['center'], 5, (0, 0, 255), -1)

        # Детекция 5x5
        corners_5, ids_5, _ = self.detector_5x5.detectMarkers(frame)
        if ids_5 is not None and len(ids_5) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners_5, ids_5)
            for corner, marker_id in zip(corners_5, ids_5.flatten()):
                info = self._compute_marker_info(corner, marker_id, '5x5')
                markers.append(info)
                cv2.circle(frame, info['center'], 5, (0, 255, 0), -1)

        return frame, markers
