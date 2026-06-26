import time
import math
from constants import TIMEOUT_POINT_REACHED

class Navigator:
    def __init__(self, drone_controller):
        self.drone = drone_controller
        self.target_point_sent = False
        self.command_sent_time = 0.0
        self.point_reached_time = 0.0
        self.current_target_pos = (0.0, 0.0)
        self.point_was_reached = False

    def send_point(self, x: float, y: float, z: float = 2.0, yaw: float = 0.0):
        self.drone.go_to_point(x, y, z, yaw)
        self.target_point_sent = True
        self.command_sent_time = time.time()
        self.point_reached_time = 0.0
        self.point_was_reached = False
        self.current_target_pos = (x, y)
        print(f"[Navigator] Команда: лететь в ({x:.2f}, {y:.2f}, {z:.2f})")

    def has_reached_point(self) -> bool:
        if not self.target_point_sent:
            return False
        if time.time() - self.command_sent_time < TIMEOUT_POINT_REACHED:
            return False
        # Используем метод дрона для проверки достижения
        return self.drone.point_reached()

    def mark_reached(self):
        if not self.point_was_reached:
            self.point_was_reached = True
            self.point_reached_time = time.time()
            print(f"[Navigator] Достигли точки {self.current_target_pos}")

    def reset_target(self):
        self.target_point_sent = False
        self.point_reached_time = 0.0
        self.point_was_reached = False
