import numpy as np

class PositionController:
    """Класс для вычисления скоростей центрирования на маркере."""
    def __init__(self, target_distance=1.2, kp_xy=0.25, kp_z=0.20, max_speed=0.3, dead_zone=0.02):
        self.target_distance = target_distance
        self.kp_xy = kp_xy
        self.kp_z = kp_z
        self.max_speed = max_speed
        self.dead_zone = dead_zone
        self.center_threshold = 0.05  # Порог для считания, что дрон отцентрирован

    def compute_centering_velocities(self, marker_coords):
        """
        Вычисляет скорости для центрирования.
        marker_coords: [x, y, z] в системе камеры (x - влево/вправо, y - вверх/вниз, z - вперед/назад)
        Возвращает: (vx, vy, vz, yaw_rate)
        """
        if marker_coords is None:
            return (0.0, 0.0, 0.0, 0.0)

        x, y, z = marker_coords

        # Ошибки с учетом мертвой зоны
        err_x = 0.0 if abs(x) < self.dead_zone else x
        err_y = 0.0 if abs(y) < self.dead_zone else y
        err_z = 0.0 if abs(z - self.target_distance) < self.dead_zone else (z - self.target_distance)

        # P-регулятор
        vx = -self.kp_xy * err_x  # Движение влево/вправо
        vy = -self.kp_xy * err_y  # Движение вверх/вниз
        vz = self.kp_z * err_z    # Движение вперед/назад (поддержание дистанции)

        # Ограничение максимальных скоростей
        vx = float(np.clip(vx, -self.max_speed, self.max_speed))
        vy = float(np.clip(vy, -self.max_speed, self.max_speed))
        vz = float(np.clip(vz, -self.max_speed, self.max_speed))

        return (vx, vy, vz, 0.0)

    def is_centered(self, marker_coords):
        """Проверяет, находится ли дрон в центре на нужном расстоянии."""
        if marker_coords is None:
            return False
        x, y, z = marker_coords
        return (abs(x) < self.center_threshold and 
                abs(y) < self.center_threshold and 
                abs(z - self.target_distance) < self.center_threshold)
