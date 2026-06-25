import numpy as np
import time

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

    def is_centered(self, marker):
        """Проверяет, отцентрирован ли дрон. Работает даже если coords=None."""
        if marker is None:
            return False
        
        # 1. Проверка по пикселям (X и Y) - всегда работает
        mx, my = marker['center']
        cx, cy = self.frame_w / 2, self.frame_h / 2
        is_centered_px = (abs(mx - cx) < self.center_threshold_px and
                          abs(my - cy) < self.center_threshold_px)
        
        # 2. Проверка по Z (в метрах) - работает только если coords есть
        if marker['coords'] is not None:
            z_3d = marker['coords'][2]
            is_centered_z = abs(z_3d - self.target_z) < self.center_threshold_z
            return is_centered_px and is_centered_z
        else:
            return is_centered_px

class SimpleCenteringLogic:
    """
    Простейший центровщик, работающий по принципу:
    - если маркер смещён от центра кадра, дрон движется с фиксированной скоростью
      в сторону уменьшения ошибки;
    - если ошибка меньше dead_zone, скорость = 0.
    """
    def __init__(self, frame_w=640, frame_h=480):
        self.frame_w = frame_w
        self.frame_h = frame_h
        # Настройки (можно менять прямо из миссии)
        self.dead_zone = 60.0          # пикселей
        self.speed = 0.03              # м/с
        # Знаки осей – если дрон едет не туда, меняем здесь
        self.sign_x = 1.0              # горизонталь (вправо/влево)
        self.sign_y = 1.0              # вертикаль (вверх/вниз)

    def compute_speeds(self, marker):
        """
        Принимает словарь маркера (обязательно поле 'center').
        Возвращает (vx, vy, vz, yaw_rate).
        vx всегда 0 (движение вперёд/назад не используется).
        """
        if marker is None or 'center' not in marker:
            return (0.0, 0.0, 0.0, 0.0)

        mx, my = marker['center']
        cx = self.frame_w / 2
        cy = self.frame_h / 2

        err_x = mx - cx   # >0 если маркер правее центра
        err_y = my - cy   # >0 если маркер ниже центра

        # Горизонтальная коррекция (vy – движение влево/вправо)
        if abs(err_x) < self.dead_zone:
            vy = 0.0
        else:
            vy = -self.sign_x * self.speed if err_x > 0 else self.sign_x * self.speed

        # Вертикальная коррекция (vz – движение вверх/вниз)
        if abs(err_y) < self.dead_zone:
            vz = 0.0
        else:
            vz = -self.sign_y * self.speed if err_y > 0 else self.sign_y * self.speed

        # Ограничим скорость (на всякий случай)
        vy = max(-self.speed, min(self.speed, vy))
        vz = max(-self.speed, min(self.speed, vz))

        return (0.0, vy, vz, 0.0)

    def is_centered(self, marker):
        """Проверяет, находится ли маркер в пределах мёртвой зоны."""
        if marker is None or 'center' not in marker:
            return False
        mx, my = marker['center']
        cx = self.frame_w / 2
        cy = self.frame_h / 2
        return abs(mx - cx) < self.dead_zone and abs(my - cy) < self.dead_zone

class StepCenteringController:
    """
    Пошаговый центровщик: двигается короткими импульсами с паузами,
    чтобы маркер не вылетал из кадра.
    """
    def __init__(self, frame_w=640, frame_h=480):
        self.frame_w = frame_w
        self.frame_h = frame_h
        
        # Параметры (можно менять извне)
        self.dead_zone = 80.0                # пикселей для центрирования
        self.step_duration = 0.15            # секунд длительность рывка
        self.step_speed = 0.025              # м/с скорость рывка
        self.wait_after_step = 0.4           # секунд пауза после рывка
        self.max_steps = 60                  # максимум шагов до таймаута
        self.required_stable_frames = 10     # кадров стабильности для завершения
        
        # Знаки осей (менять при необходимости)
        self.sign_x = 1.0
        self.sign_y = 1.0
        
        # Внутреннее состояние
        self.reset()
    
    def reset(self):
        """Сброс состояния для новой цели."""
        self.step_phase = 0          # 0 = стоим, 1 = двигаемся, 2 = пауза
        self.step_start_time = 0.0
        self.current_step_direction = (0.0, 0.0)
        self.step_counter = 0
        self.stable_frames = 0
        self.centering_start_time = time.time()
        self.is_centered_flag = False   # флаг, что центровка достигнута и стабильна
    
    def update(self, marker):
        """
        Основной метод: принимает маркер (словарь с полем 'center'),
        возвращает (vx, vy, vz, yaw_rate, centered_done).
        centered_done = True, когда центровка успешно завершена (стабильно в центре).
        """
        if marker is None or 'center' not in marker:
            # Если маркер потерян – останавливаемся и возвращаем centred_done=False
            return (0.0, 0.0, 0.0, 0.0, False)
        
        mx, my = marker['center']
        cx = self.frame_w / 2
        cy = self.frame_h / 2
        err_x = mx - cx   # >0 если правее
        err_y = my - cy   # >0 если ниже
        
        # Проверка центрирования
        if abs(err_x) < self.dead_zone and abs(err_y) < self.dead_zone:
            self.stable_frames += 1
            if self.stable_frames >= self.required_stable_frames:
                self.is_centered_flag = True
                return (0.0, 0.0, 0.0, 0.0, True)  # центровка завершена
            else:
                # Стоим, накапливаем стабильность
                return (0.0, 0.0, 0.0, 0.0, False)
        else:
            self.stable_frames = 0   # сброс стабильности, если маркер сместился
        
        now = time.time()
        
        # --- Пошаговый автомат ---
        if self.step_phase == 0:
            # Стоим – определяем направление шага
            vy = 0.0
            vz = 0.0
            if abs(err_x) > self.dead_zone:
                vy = -self.sign_x * self.step_speed if err_x > 0 else self.sign_x * self.step_speed
            if abs(err_y) > self.dead_zone:
                vz = -self.sign_y * self.step_speed if err_y > 0 else self.sign_y * self.step_speed
            
            if vy == 0.0 and vz == 0.0:
                # Ошибка меньше dead_zone, но stable_frames не накопился – стоим
                return (0.0, 0.0, 0.0, 0.0, False)
            
            self.current_step_direction = (vy, vz)
            self.step_start_time = now
            self.step_phase = 1
            return (0.0, 0.0, 0.0, 0.0, False)  # на следующем вызове начнём движение
        
        elif self.step_phase == 1:
            # Двигаемся в течение step_duration
            elapsed = now - self.step_start_time
            if elapsed < self.step_duration:
                vy, vz = self.current_step_direction
                return (0.0, vy, vz, 0.0, False)
            else:
                # Заканчиваем движение, переходим в паузу
                self.step_phase = 2
                self.step_start_time = now
                self.step_counter += 1
                return (0.0, 0.0, 0.0, 0.0, False)
        
        elif self.step_phase == 2:
            # Пауза после движения
            if now - self.step_start_time > self.wait_after_step:
                # Переходим к следующему шагу
                self.step_phase = 0
                if self.step_counter >= self.max_steps:
                    # Таймаут, считаем центровку завершённой (принудительно)
                    return (0.0, 0.0, 0.0, 0.0, True)
            # Стоим
            return (0.0, 0.0, 0.0, 0.0, False)
        
        # Защита
        return (0.0, 0.0, 0.0, 0.0, False)
    
    def is_centered(self, marker):
        """Простая проверка центрирования без стабильности."""
        if marker is None:
            return False
        mx, my = marker['center']
        cx = self.frame_w / 2
        cy = self.frame_h / 2
        return abs(mx - cx) < self.dead_zone and abs(my - cy) < self.dead_zone
