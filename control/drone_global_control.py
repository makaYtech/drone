from pioneer_sdk import Pioneer
import time
import numpy as np
from config import FORWARD_SPEED
from control.flight_logger import FlightLogger

class DroneGlobalController:
    def __init__(self, ip, port, simulator=True):
        self.drone = Pioneer(ip=ip, mavlink_port=port, simulator=simulator)
        self._last_hold_time = 0
        self.inertia_compensating = False
        self.inertia_end_time = 0.0
        self.logger = FlightLogger("flight_log.csv")

        self._pos_history = []
        self._pos_history_maxlen = 10

    # ---- Основные команды ----
    def go_to_point(self, x, y, z, yaw=0.0):
        self.drone.go_to_local_point(x, y, z, yaw)
        self.logger.log_goto(x, y, z, yaw)

    def point_reached(self):
        return self.drone.point_reached()

    def set_manual_speed(self, vx=0, vy=0, vz=0, yaw_rate=0):
        self.drone.set_manual_speed_body_fixed(vx, vy, vz, yaw_rate)
        self.logger.log_speed(vx, vy, vz, yaw_rate)

    def arm(self):
        self.drone.arm()
        self.logger.log_event("ARM", "Дрон armed")

    def takeoff(self, altitude=5.0):
        self.drone.takeoff()
        self.logger.log_event("TAKEOFF", f"Высота {altitude}")
        time.sleep(2)

    def land(self):
        self.logger.log_event("LAND", "Команда на посадку")
        self.drone.land()
        self.drone.disarm()
        # Логгер НЕ закрываем здесь — он будет закрыт в конце программы

    def close_logger(self):
        self.logger.close()

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

    # ---- Телеметрия ----
    def get_position(self):
        pos = self.drone.get_local_position_lps(get_last_received=False)
        if pos is not None:
            return pos[0], pos[1], pos[2]
        return None

    def get_yaw(self):
        return self.drone.get_yaw()

    def get_battery_voltage(self):
        return self.drone.get_battery_status()

    def get_autopilot_state(self):
        return self.drone.get_autopilot_state()

    def is_armed(self):
        state = self.get_autopilot_state()
        return state == "ARMED" or state == "TAKEOFF" or state == "FLYING"

    def is_in_air(self):
        state = self.get_autopilot_state()
        return state == "TAKEOFF" or state == "FLYING" or state == "LANDING"

    # ---- Проверка стабильности позиции ----
    def update_position_history(self):
        pos = self.get_position()
        if pos is not None:
            now = time.time()
            self._pos_history.append((now, pos[0], pos[1], pos[2]))
            if len(self._pos_history) > self._pos_history_maxlen:
                self._pos_history.pop(0)

    def is_position_stable(self, threshold=0.05, duration=0.5):
        now = time.time()
        cutoff = now - duration
        recent = [(t, x, y, z) for (t, x, y, z) in self._pos_history if t >= cutoff]
        if len(recent) < 2:
            return False
        xs = [x for _, x, _, _ in recent]
        ys = [y for _, _, y, _ in recent]
        zs = [z for _, _, _, z in recent]
        dx = max(xs) - min(xs)
        dy = max(ys) - min(ys)
        dz = max(zs) - min(zs)
        return (dx < threshold and dy < threshold and dz < threshold)
