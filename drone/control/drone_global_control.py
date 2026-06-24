from pioneer_sdk import Pioneer
import time
import numpy as np
from config import DIST_FAR, DIST_NEAR, FORWARD_SPEED, YAW_SPEED

class DroneGlobalController:
    def __init__(self, ip, port, simulator=True):
        self.drone = Pioneer(ip=ip, mavlink_port=port, simulator=simulator)
        self.armed = False
        self.in_air = False
        self._last_hold_time = 0
        
        # Для гашения инерции
        self.inertia_compensating = False
        self.inertia_end_time = 0.0

    def arm(self):
        if not self.armed:
            self.drone.arm()
            self.armed = True

    def takeoff(self, altitude=2.0):
        if not self.in_air:
            self.drone.takeoff()
            self.in_air = True
            time.sleep(2)

    def land(self):
        if self.in_air:
            self.drone.land()
            self.drone.disarm()
            self.in_air = False

    def set_manual_speed(self, vx=0, vy=0, vz=0, yaw_rate=0):
        """Отправляет команду скорости."""
        self.drone.set_manual_speed_body_fixed(vx, vy, vz, yaw_rate)

    def hold_position(self):
        """Удерживает позицию — отправляет команду не чаще раза в секунду."""
        now = time.time()
        if now - self._last_hold_time > 0.5:
            self.drone.go_to_local_point_body_fixed(x=0, y=0, z=0, yaw=0)
            self._last_hold_time = now

    def stop_with_inertia(self):
        """Запускает процесс гашения инерции (кратковременный полёт назад)."""
        if not self.inertia_compensating:
            self.set_manual_speed(vx=0, vy=-0.5 * FORWARD_SPEED, vz=0, yaw_rate=0)
            self.inertia_compensating = True
            self.inertia_end_time = time.time() + 0.3

    def update_inertia(self):
        """Вызывается в главном цикле для поддержания гашения инерции."""
        if self.inertia_compensating:
            if time.time() >= self.inertia_end_time:
                self.set_manual_speed(vx=0, vy=0, vz=0, yaw_rate=0)
                self.inertia_compensating = False
            else:
                self.set_manual_speed(vx=0, vy=-0.5 * FORWARD_SPEED, vz=0, yaw_rate=0)