from pioneer_sdk import Pioneer
import time
import numpy as np
from config import DIST_FAR, DIST_NEAR, FORWARD_SPEED, YAW_SPEED

class DroneGlobalController:
    def __init__(self, ip, port, simulator=True):
        self.drone = Pioneer(ip=ip, mavlink_port=port, simulator=simulator)
        self.armed = False
        self.in_air = False

    def arm(self):
        if not self.armed:
            self.drone.arm()
            self.armed = True

    def takeoff(self, altitude=2.0):
        if not self.in_air:
            self.drone.takeoff()
            self.drone.go_to_local_point(0, 0, altitude, 0)
            self.in_air = True
            time.sleep(2)

    def land(self):
        if self.in_air:
            self.drone.land()
            self.in_air = False

    def set_manual_speed(self, vx=0, vy=0, vz=0, yaw_rate=0):
        self.drone.set_manual_speed_body_fixed(vx, vy, vz, yaw_rate)

    def hold_position(self):
        self.drone.go_to_local_point_body_fixed(x=0, y=0, z=0, yaw=0)

    def compute_control(self, coords, x_center, frame_width):
        if coords is None or x_center is None:
            return (0, 0, 0, 0), False

        distance = np.linalg.norm(coords)
        vy = 0.0
        yaw_rate = 0.0

        if distance > DIST_FAR:
            vy = FORWARD_SPEED
        elif distance < DIST_NEAR:
            vy = -FORWARD_SPEED

        if x_center is not None and frame_width is not None:
            if x_center < frame_width / 3:
                yaw_rate = -YAW_SPEED
            elif x_center > frame_width * 2 / 3:
                yaw_rate = YAW_SPEED

        if vy != 0 or yaw_rate != 0:
            return (0, vy, 0, yaw_rate), True
        else:
            return (0, 0, 0, 0), False
