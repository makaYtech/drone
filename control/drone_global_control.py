from pioneer_sdk import Pioneer
import time
import numpy as np
from config import FORWARD_SPEED
from control.flight_logger import FlightLogger

class DroneGlobalController:
    def __init__(self, ip, port, simulator=True):
        self.drone = Pioneer(ip=ip, mavlink_port=port, simulator=simulator)
        self.armed = False
        self.in_air = False
        self._last_hold_time = 0
        
        self.inertia_compensating = False
        self.inertia_end_time = 0.0
        
        # Инициализация логгера
        self.logger = FlightLogger("flight_log.csv")

    def go_to_point(self, x, y, z, yaw=0.0):
        self.drone.go_to_local_point(x, y, z, yaw)
        self.logger.log_goto(x, y, z, yaw)

    def point_reached(self):
        return self.drone.point_reached()

    def arm(self):
        if not self.armed:
            self.drone.arm()
            self.armed = True
            self.logger.log_event("ARM", "Дрон armed")

    def takeoff(self, altitude=5.0):
        if not self.in_air:
            self.drone.takeoff()
            self.in_air = True
            self.logger.log_event("TAKEOFF", f"Высота {altitude}")
            time.sleep(2)

    def land(self):
        if self.in_air:
            self.logger.log_event("LAND", "Команда на посадку")
            self.drone.land()
            self.drone.disarm()
            self.in_air = False
            self.logger.close()

    def set_manual_speed(self, vx=0, vy=0, vz=0, yaw_rate=0):
        self.drone.set_manual_speed_body_fixed(vx, vy, vz, yaw_rate)
        self.logger.log_speed(vx, vy, vz, yaw_rate)

    def hold_position(self):
        now = time.time()
        if now - self._last_hold_time > 0.5:
            self.drone.go_to_local_point_body_fixed(x=0, y=0, z=0, yaw=0)
            self._last_hold_time = now

    def stop_with_inertia(self):
        self.set_manual_speed(vx=0, vy=-0.5 * FORWARD_SPEED, vz=0, yaw_rate=0)
        time.sleep(0.3)
        self.set_manual_speed(vx=0, vy=0, vz=0, yaw_rate=0)

    def update_inertia(self):
        if self.inertia_compensating:
            if time.time() >= self.inertia_end_time:
                self.set_manual_speed(vx=0, vy=0, vz=0, yaw_rate=0)
                self.inertia_compensating = False
            else:
                self.set_manual_speed(vx=0, vy=-0.5 * FORWARD_SPEED, vz=0, yaw_rate=0)
