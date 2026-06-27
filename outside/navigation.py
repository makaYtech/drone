import time
import math
from constants import TIMEOUT_POINT_REACHED   # больше не используется, но оставлю на случай

class Navigator:
    def __init__(self, drone_controller):
        self.drone = drone_controller
        self.current_target_pos = (0.0, 0.0)

    def send_point(self, x: float, y: float, z: float = 2.0, yaw: float = 0.0):
        self.drone.go_to_point(x, y, z, yaw)
        self.current_target_pos = (x, y)
        print(f"[Navigator] Команда: лететь в ({x:.2f}, {y:.2f}, {z:.2f})")

    def has_reached_point(self) -> bool:
        """Использует встроенный метод point_reached() из SDK."""
        return self.drone.point_reached()

    def reset_target(self):
        """Не требуется, так как point_reached() автоматически сбрасывается после подтверждения."""
        pass
