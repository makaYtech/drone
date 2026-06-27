import random
import time
from constants import (
    MIN_CENTERING_STEP, MAX_CENTERING_STEP,
    CENTERING_TOLERANCE_X, CENTERING_TOLERANCE_Y,
    CENTERING_MAX_ATTEMPTS, REQUIRED_STABLE_FRAMES
)

class CenteringController:
    def __init__(self, frame_w=640, frame_h=480):
        self.frame_w = frame_w
        self.frame_h = frame_h

        self.stable_frames = 0
        self.attempts = 0
        self.bad_directions = []          # все плохие направления (глобально)
        self.last_bad_directions = []     # последние 3 плохих направления
        self.last_good_direction = (0.0, 0.0)
        self.good_directions_history = []
        self.current_move_direction = (0.0, 0.0)
        self.last_marker_pos = None
        self.anchor_x = 0.0
        self.anchor_y = 0.0
        self.current_step = MIN_CENTERING_STEP
        self.last_results = []

    def reset(self, anchor_x, anchor_y):
        self.stable_frames = 0
        self.attempts = 0
        self.bad_directions = []
        self.last_bad_directions = []
        self.last_good_direction = (0.0, 0.0)
        self.good_directions_history = []
        self.current_move_direction = (0.0, 0.0)
        self.last_marker_pos = None
        self.anchor_x = anchor_x
        self.anchor_y = anchor_y
        self.current_step = MIN_CENTERING_STEP
        self.last_results = []

    def is_marker_centered(self, marker):
        if marker is None or 'center' not in marker:
            return False
        mx, my = marker['center']
        cx, cy = self.frame_w / 2, self.frame_h / 2
        return (abs(mx - cx) < CENTERING_TOLERANCE_X and
                abs(my - cy) < CENTERING_TOLERANCE_Y)

    def get_marker_center(self, marker):
        if marker is None or 'center' not in marker:
            return None
        return marker['center']

    def get_centering_step(self, marker):
        if marker is None:
            return self.current_step
        mx, my = marker['center']
        cx, cy = self.frame_w / 2, self.frame_h / 2
        dist = ((mx - cx) ** 2 + (my - cy) ** 2) ** 0.5
        max_dist = ((self.frame_w/2) ** 2 + (self.frame_h/2) ** 2) ** 0.5
        ratio = min(1.0, dist / max_dist)
        base_step = MIN_CENTERING_STEP + (MAX_CENTERING_STEP - MIN_CENTERING_STEP) * ratio
        step = max(base_step, self.current_step * 0.5)
        step = min(step, MAX_CENTERING_STEP)
        return round(step, 2)

    def choose_direction(self, step, err_x=None, err_y=None):
        """
        Выбирает направление движения.
        Если два последних направления были плохими, пытается пойти в противоположную сумму.
        """
        # Если ошибка передана и она велика – используем принудительное направление
        if err_x is not None and err_y is not None:
            if abs(err_x) > 150 or abs(err_y) > 150:
                dx = step if err_x > 0 else -step
                dy = step if err_y > 0 else -step
                if abs(err_x) < 100:
                    dx = 0
                if abs(err_y) < 100:
                    dy = 0
                if (dx, dy) not in self.bad_directions:
                    return (dx, dy)
                # Если направление по ошибке уже плохое – пробуем альтернативы
                alt_dirs = []
                if dx != 0:
                    alt_dirs.append((-dx, dy))
                if dy != 0:
                    alt_dirs.append((dx, -dy))
                for d in alt_dirs:
                    if d not in self.bad_directions:
                        return d

        all_dirs = [
            (step, 0), (-step, 0), (0, step), (0, -step),
            (step, step), (step, -step), (-step, step), (-step, -step)
        ]

        # --- НОВАЯ ЛОГИКА: комбинирование двух последних плохих направлений ---
        if len(self.last_bad_directions) >= 2:
            last1 = self.last_bad_directions[-1]
            last2 = self.last_bad_directions[-2]
            # Суммируем векторы и берём противоположное
            sum_dx = last1[0] + last2[0]
            sum_dy = last1[1] + last2[1]
            # Если сумма не нулевая, нормализуем до длины step
            if abs(sum_dx) > 0.01 or abs(sum_dy) > 0.01:
                norm = (sum_dx**2 + sum_dy**2)**0.5
                opp_dx = -sum_dx / norm * step
                opp_dy = -sum_dy / norm * step
                # Округляем до 2 знаков
                opp_dx = round(opp_dx, 2)
                opp_dy = round(opp_dy, 2)
                # Проверяем, что направление не в плохих
                if (opp_dx, opp_dy) not in self.bad_directions:
                    print(f"[Centering] Комбинирую два плохих: {last1}+{last2} -> противоположное ({opp_dx:.2f}, {opp_dy:.2f})")
                    return (opp_dx, opp_dy)

        # Убираем плохие направления
        candidates = [d for d in all_dirs if d not in self.bad_directions]
        if not candidates:
            print("[Centering] Все направления плохие, сбрасываю историю.")
            self.bad_directions = []
            candidates = all_dirs

        # Если последнее успешное – с вероятностью 60% пробуем противоположное
        if self.last_good_direction != (0.0, 0.0):
            opposite = (-self.last_good_direction[0], -self.last_good_direction[1])
            if opposite in candidates and random.random() < 0.6:
                print(f"[Centering] Выбираю противоположное {opposite} (последнее успешное {self.last_good_direction})")
                return opposite

        chosen = random.choice(candidates)
        print(f"[Centering] Выбираю случайное направление {chosen} из {len(candidates)}")
        return chosen

    def analyze_after_move(self, marker, current_x, current_y, move_direction):
        """
        Анализирует результат движения.
        Возвращает ('good'/'bad'/'centered'/'lost', new_anchor_x, new_anchor_y)
        """
        self.current_move_direction = move_direction

        if marker is None:
            self.last_results.append((move_direction, 'lost'))
            if len(self.last_results) > 3:
                self.last_results.pop(0)
            return 'lost', None, None

        if self.is_marker_centered(marker):
            self.last_results.append((move_direction, 'centered'))
            if len(self.last_results) > 3:
                self.last_results.pop(0)
            return 'centered', None, None

        new_marker_pos = self.get_marker_center(marker)
        if self.last_marker_pos is None:
            self.last_results.append((move_direction, 'bad'))
            if len(self.last_results) > 3:
                self.last_results.pop(0)
            return 'bad', None, None

        old_dist = self._distance_to_center(self.last_marker_pos)
        new_dist = self._distance_to_center(new_marker_pos)

        if new_dist < old_dist:
            self.last_good_direction = move_direction
            self.good_directions_history.append(move_direction)
            if len(self.good_directions_history) > 3:
                self.good_directions_history.pop(0)
            self.current_step = min(MAX_CENTERING_STEP, self.current_step * 1.2)
            new_anchor_x, new_anchor_y = current_x, current_y
            self.attempts = 0
            self.last_bad_directions = []
            print(f"[Centering] Хорошее направление! Расстояние: {old_dist:.1f} -> {new_dist:.1f}, шаг увеличен до {self.current_step:.3f}")
            self.last_results.append((move_direction, 'good'))
            if len(self.last_results) > 3:
                self.last_results.pop(0)
            return 'good', new_anchor_x, new_anchor_y
        else:
            if move_direction != (0.0, 0.0):
                self.bad_directions.append(move_direction)
                self.last_bad_directions.append(move_direction)
                if len(self.last_bad_directions) > 3:
                    self.last_bad_directions.pop(0)
            self.attempts += 1
            self.current_step = max(MIN_CENTERING_STEP, self.current_step * 0.8)
            print(f"[Centering] Плохое направление. Расстояние: {old_dist:.1f} -> {new_dist:.1f}, шаг уменьшен до {self.current_step:.3f}")
            self.last_results.append((move_direction, 'bad'))
            if len(self.last_results) > 3:
                self.last_results.pop(0)
            return 'bad', None, None

    def _distance_to_center(self, pos):
        if pos is None:
            return float('inf')
        mx, my = pos
        cx, cy = self.frame_w / 2, self.frame_h / 2
        return ((mx - cx) ** 2 + (my - cy) ** 2) ** 0.5

    def check_centered_stable(self, marker):
        if self.is_marker_centered(marker):
            self.stable_frames += 1
            if self.stable_frames >= REQUIRED_STABLE_FRAMES:
                return True, self.stable_frames
        else:
            if marker is not None:
                mx, my = marker['center']
                cx, cy = self.frame_w / 2, self.frame_h / 2
                if abs(mx - cx) < 100 and abs(my - cy) < 100:
                    self.stable_frames += 1
                    if self.stable_frames >= REQUIRED_STABLE_FRAMES:
                        return True, self.stable_frames
                else:
                    self.stable_frames = 0
            else:
                self.stable_frames = 0
        return False, self.stable_frames

    def get_current_error(self, marker):
        if marker is None or 'center' not in marker:
            return None, None
        mx, my = marker['center']
        cx, cy = self.frame_w / 2, self.frame_h / 2
        return mx - cx, my - cy
