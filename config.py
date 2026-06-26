import numpy as np
import cv2

# ====== Сетевые настройки ======
IP = '127.0.0.1'
PORT = 8000
PORT_CAM = 18000

# ====== Параметры ArUco ======
ARUCO_DICT = cv2.aruco.DICT_4X4_50
SIZE_OF_MARKER = 0.1

# ====== Параметры калибровки камеры ======
CAMERA_MATRIX = np.array([
    [600,   0, 320],
    [  0, 600, 240],
    [  0,   0,   1]
], dtype=np.float64)
DIST_COEFFS = np.zeros((5, 1), dtype=np.float64)

# ====== Параметры управления (скорости, дистанции) ======
DIST_FAR = 1.2
DIST_NEAR = 0.5
FORWARD_SPEED = 0.4
YAW_SPEED = 0.4
