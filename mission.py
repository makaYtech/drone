import json
import time
import math
import numpy as np
from typing import List, Dict

class Mission:
    def __init__(self, drone_controller, initial_yaw: float = 0, target_groups: int = 4):
        self.drone = drone_controller
        self.target_groups = target_groups
        self.collected = []
        self.map_data = []

        self.SQUARE_SIZE = 10.0
        self.SPIRAL_STEP = 1.0
        self.CALIBRATION_DISTANCE = 10.0

        # Состояния
        self.STATE_WAITING = "waiting"
        self.STATE_GO_TO_ORIGIN = "go_to_origin"
        self.STATE_CALIBRATION = "calibration"
        self.STATE_RETURN_TO_ORIGIN = "return_to_origin"
        self.STATE_SCAN_SQUARE = "scan_square"
        self.STATE_DECIDE_NEXT = "decide_next"
        self.STATE_MOVE_TO_SQUARE = "move_to_square"
        self.STATE_RETURN_HOME = "return_home"
        self.STATE_DONE = "done"

        self.state = self.STATE_WAITING
        self.state_start_time = time.time()

        self.spiral_points = []
        self.current_spiral_idx = 0
        self.recorded_pairs = set()

        self.target_point_sent = False
        self.command_sent_time = 0.0
        self.point_reached_time = 0.0
        self.current_target_pos = (0.0, 0.0)
        self.point_was_reached = False

        self.current_square = (0, 0)
        self.last_square_with_markers = (0, 0)
        self.direction = "forward"
        self.squares_with_markers = []
        self.has_been_backward = False

    def start(self):
        self.state = self.STATE_WAITING
        self.state_start_time = time.time()
        print("[Mission] Миссия запущена, ожидание 2 секунды...")

    def update(self, markers: List[Dict]) -> str:
        if self.state == self.STATE_DONE:
            return "done"

        if self.state == self.STATE_WAITING:
            self._handle_waiting()
        elif self.state == self.STATE_GO_TO_ORIGIN:
            self._handle_go_to_origin()
        elif self.state == self.STATE_CALIBRATION:
            self._handle_calibration()
        elif self.state == self.STATE_RETURN_TO_ORIGIN:
            self._handle_return_to_origin()
        elif self.state == self.STATE_SCAN_SQUARE:
            self._handle_scan_square(markers)
        elif self.state == self.STATE_DECIDE_NEXT:
            self._handle_decide_next()
        elif self.state == self.STATE_MOVE_TO_SQUARE:
            self._handle_move_to_square(markers)
        elif self.state == self.STATE_RETURN_HOME:
            self._handle_return_home()

        return self.state

    def _send_point(self, x, y):
        self.drone.drone.go_to_local_point(x, y, 2, 0)
        self.target_point_sent = True
        self.command_sent_time = time.time()
        self.point_reached_time = 0.0
        self.point_was_reached = False
        self.current_target_pos = (x, y)
        print(f"[Mission] Команда: лететь в ({x:.2f}, {y:.2f})")

    def _has_reached_point(self):
        if not self.target_point_sent:
            return False
        if time.time() - self.command_sent_time < 2.0:
            return False
        return self.drone.drone.point_reached()

    def _handle_waiting(self):
        if time.time() - self.state_start_time > 2.0:
            print("[Mission] Летим в (0, 0)...")
            self.state = self.STATE_GO_TO_ORIGIN
            self._send_point(0, 0)

    def _handle_go_to_origin(self):
        if self._has_reached_point():
            print("[Mission] Прибыли в (0, 0). Начинаем калибровку...")
            self.state = self.STATE_CALIBRATION
            self._send_point(0, self.CALIBRATION_DISTANCE)

    def _handle_calibration(self):
        if self._has_reached_point():
            print(f"[Mission] Прибыли в (0, {self.CALIBRATION_DISTANCE}). Возвращаемся...")
            self.state = self.STATE_RETURN_TO_ORIGIN
            self._send_point(0, 0)

    def _handle_return_to_origin(self):
        if self._has_reached_point():
            print("[Mission] Вернулись в (0, 0). Начинаем сканирование...")
            self.current_square = (0, 0)
            self._generate_spiral_points(self.current_square)
            self.state = self.STATE_SCAN_SQUARE
            self.current_spiral_idx = 0
            self.target_point_sent = False

    def _generate_spiral_points(self, square_offset):
        """Генерирует НЕПРЕРЫВНУЮ спираль от центра к краям.
        Каждое кольцо начинается там, где закончилось предыдущее.
        Все переходы строго по одной оси (без диагоналей)."""
        x_off, y_off = square_offset
        cx = x_off + self.SQUARE_SIZE / 2
        cy = y_off + self.SQUARE_SIZE / 2
        step = self.SPIRAL_STEP
        num_rings = int((self.SQUARE_SIZE / 2.0) / step)

        points = [(round(cx, 2), round(cy, 2))]

        for ring in range(1, num_rings + 1):
            x_min = round(max(x_off, cx - ring * step), 2)
            x_max = round(min(x_off + self.SQUARE_SIZE, cx + ring * step), 2)
            y_min = round(max(y_off, cy - ring * step), 2)
            y_max = round(min(y_off + self.SQUARE_SIZE, cy + ring * step), 2)

            last_x, last_y = points[-1]

            # Переход: вверх до y_max нового кольца
            while last_y < y_max - 0.01:
                last_y = round(min(last_y + step, y_max), 2)
                points.append((last_x, last_y))

            # Переход: влево до x_min нового кольца
            while last_x > x_min + 0.01:
                last_x = round(max(last_x - step, x_min), 2)
                points.append((last_x, last_y))

            # Теперь мы в (x_min, y_max) — верхний-левый угол кольца

            # Вправо по верхней стороне
            while last_x < x_max - 0.01:
                last_x = round(min(last_x + step, x_max), 2)
                points.append((last_x, last_y))

            # Вниз по правой стороне
            while last_y > y_min + 0.01:
                last_y = round(max(last_y - step, y_min), 2)
                points.append((last_x, last_y))

            # Влево по нижней стороне
            while last_x > x_min + 0.01:
                last_x = round(max(last_x - step, x_min), 2)
                points.append((last_x, last_y))

            # Вверх по левой стороне (не доходя до y_max, чтобы не замкнуть)
            target_y = round(y_max - step, 2) if ring < num_rings else y_max
            while last_y < target_y - 0.01:
                last_y = round(min(last_y + step, target_y), 2)
                points.append((last_x, last_y))

        # Убираем дубликаты, сохраняя порядок
        unique = []
        seen = set()
        for p in points:
            r = (round(p[0], 2), round(p[1], 2))
            if r not in seen:
                seen.add(r)
                unique.append(r)

        # Гарантируем наличие углов квадрата
        corners = [
            (round(x_off, 2), round(y_off, 2)),
            (round(x_off, 2), round(y_off + self.SQUARE_SIZE, 2)),
            (round(x_off + self.SQUARE_SIZE, 2), round(y_off, 2)),
            (round(x_off + self.SQUARE_SIZE, 2), round(y_off + self.SQUARE_SIZE, 2))
        ]
        for c in corners:
            if c not in seen:
                unique.append(c)

        self.spiral_points = unique
        print(f"[Mission] Квадрат {square_offset}: {len(self.spiral_points)} точек")

    def _find_straight_line_end(self, start_idx):
        """Если 3+ точек идут строго по одной оси в одном направлении —
        возвращает индекс последней. Иначе возвращает start_idx."""
        if start_idx >= len(self.spiral_points) - 2:
            return start_idx

        p0 = self.spiral_points[start_idx]
        p1 = self.spiral_points[start_idx + 1]

        dx = round(p1[0] - p0[0], 2)
        dy = round(p1[1] - p0[1], 2)

        # Определяем ось движения
        if abs(dx) > 0.01 and abs(dy) < 0.01:
            axis = 'x'
            sign = 1 if dx > 0 else -1
        elif abs(dy) > 0.01 and abs(dx) < 0.01:
            axis = 'y'
            sign = 1 if dy > 0 else -1
        else:
            return start_idx  # Диагональ — не оптимизируем

        # Ищем конец прямой
        end_idx = start_idx + 1
        for i in range(start_idx + 2, len(self.spiral_points)):
            curr = self.spiral_points[i]
            prev = self.spiral_points[i - 1]

            d_x = round(curr[0] - prev[0], 2)
            d_y = round(curr[1] - prev[1], 2)

            if axis == 'x':
                # Должно быть движение ТОЛЬКО по X в том же направлении
                if abs(d_y) > 0.01 or (d_x * sign) < 0.01:
                    break
            else:
                # Должно быть движение ТОЛЬКО по Y в том же направлении
                if abs(d_x) > 0.01 or (d_y * sign) < 0.01:
                    break

            end_idx = i

        # Оптимизируем только если 3+ точек в цепочке (включая стартовую)
        if end_idx - start_idx + 1 >= 3:
            return end_idx
        else:
            return start_idx

    def _handle_scan_square(self, markers: List[Dict]):
        if not self.target_point_sent:
            if self.current_spiral_idx >= len(self.spiral_points):
                print(f"[Mission] Квадрат {self.current_square} просканирован.")
                self.state = self.STATE_DECIDE_NEXT
                return

            line_end = self._find_straight_line_end(self.current_spiral_idx)

            if line_end > self.current_spiral_idx:
                target = self.spiral_points[line_end]
                print(f"[Mission] Прямая: {self.current_spiral_idx} → {line_end} "
                      f"к ({target[0]:.2f}, {target[1]:.2f})")
                self._send_point(target[0], target[1])
                self._target_spiral_idx = line_end
            else:
                x, y = self.spiral_points[self.current_spiral_idx]
                print(f"[Mission] Спираль: точка ({x:.2f}, {y:.2f}) "
                      f"[{self.current_spiral_idx}/{len(self.spiral_points)}]")
                self._send_point(x, y)
                self._target_spiral_idx = self.current_spiral_idx
            return

        # Сканируем метки во время полёта
        self._check_and_record_markers(markers)

        if self._has_reached_point():
            if not self.point_was_reached:
                self.point_was_reached = True
                self.point_reached_time = time.time()
                print(f"[Mission] Достигли точки {self.current_target_pos}")

        if self.point_was_reached:
            if time.time() - self.point_reached_time >= 1.0:
                if hasattr(self, '_target_spiral_idx'):
                    self.current_spiral_idx = self._target_spiral_idx
                    del self._target_spiral_idx

                self.current_spiral_idx += 1
                self.target_point_sent = False
                self.point_reached_time = 0
                self.point_was_reached = False

    def _handle_decide_next(self):
        found_in_current = any(
            self._is_in_square(entry['pos'], self.current_square)
            for entry in self.collected
        )

        if found_in_current:
            self.last_square_with_markers = self.current_square
            if self.current_square not in self.squares_with_markers:
                self.squares_with_markers.append(self.current_square)
            print(f"[Mission] В квадрате {self.current_square} найдены метки! "
                  f"Продолжаем '{self.direction}'.")

            if self.direction == "forward":
                next_sq = (self.current_square[0], self.current_square[1] + self.SQUARE_SIZE)
            elif self.direction == "backward":
                next_sq = (self.current_square[0], self.current_square[1] - self.SQUARE_SIZE)
            else:
                next_sq = (self.current_square[0] + self.SQUARE_SIZE, self.current_square[1])

            self.current_square = next_sq
            self.state = self.STATE_MOVE_TO_SQUARE
        else:
            print(f"[Mission] В квадрате {self.current_square} меток нет.")

            if self.direction == "forward":
                if len(self.squares_with_markers) == 0:
                    print("[Mission] Вперёд ничего. Пробуем назад.")
                    self.current_square = (0, -int(self.SQUARE_SIZE))
                    self.direction = "backward"
                    self.has_been_backward = True
                    self.state = self.STATE_MOVE_TO_SQUARE
                else:
                    print(f"[Mission] Возврат на {self.last_square_with_markers}, бок.")
                    self.current_square = (
                        self.last_square_with_markers[0] + int(self.SQUARE_SIZE),
                        self.last_square_with_markers[1])
                    self.direction = "side"
                    self.state = self.STATE_MOVE_TO_SQUARE
            elif self.direction == "backward":
                print(f"[Mission] Возврат на {self.last_square_with_markers}, бок.")
                self.current_square = (
                    self.last_square_with_markers[0] + int(self.SQUARE_SIZE),
                    self.last_square_with_markers[1])
                self.direction = "side"
                self.state = self.STATE_MOVE_TO_SQUARE
            else:
                if not self.has_been_backward:
                    print("[Mission] Бок не дал. Возврат в (0,0), назад.")
                    self.state = self.STATE_RETURN_HOME
                    self._send_point(0, 0)
                    self._after_return_action = "backward"
                else:
                    print("[Mission] Всё проверено. Возврат в (0, 0).")
                    self.state = self.STATE_RETURN_HOME
                    self._send_point(0, 0)
                    self._after_return_action = "done"

    def _is_in_square(self, pos, square_offset):
        x_off, y_off = square_offset
        x, y = pos[0], pos[1]
        return (x_off <= x <= x_off + self.SQUARE_SIZE and
                y_off <= y <= y_off + self.SQUARE_SIZE)

    def _handle_move_to_square(self, markers: List[Dict]):
        if not self.target_point_sent:
            x_off, y_off = self.current_square
            center_x = x_off + self.SQUARE_SIZE / 2
            center_y = y_off + self.SQUARE_SIZE / 2
            print(f"[Mission] Летим в квадрат {self.current_square}: "
                  f"({center_x:.2f}, {center_y:.2f})")
            self._send_point(center_x, center_y)
            return

        # Сканируем по пути
        self._check_and_record_markers(markers)

        if self._has_reached_point():
            print(f"[Mission] Прибыли в квадрат {self.current_square}. "
                  f"Спиральное сканирование.")
            self._generate_spiral_points(self.current_square)
            self.state = self.STATE_SCAN_SQUARE
            self.current_spiral_idx = 0
            self.target_point_sent = False

    def _handle_return_home(self):
        if self._has_reached_point():
            action = getattr(self, '_after_return_action', 'done')

            if action == "backward":
                print("[Mission] Вернулись в (0, 0). Движение назад.")
                self.current_square = (0, -int(self.SQUARE_SIZE))
                self.direction = "backward"
                self.has_been_backward = True
                self._generate_spiral_points(self.current_square)
                self.state = self.STATE_SCAN_SQUARE
                self.current_spiral_idx = 0
                self.target_point_sent = False
                self._after_return_action = None
            else:
                print(f"[Mission] Миссия завершена. "
                      f"Квадратов с метками: {len(self.squares_with_markers)}")
                self.state = self.STATE_DONE

    def _check_and_record_markers(self, markers: List[Dict]):
        m4 = [m for m in markers if m['type'] == '4x4' and m['coords'] is not None]
        m5 = [m for m in markers if m['type'] == '5x5' and m['coords'] is not None]

        if m4 and m5:
            for marker4 in m4:
                for marker5 in m5:
                    pair_id = (marker4['id'], marker5['id'])
                    if pair_id not in self.recorded_pairs:
                        entry = {
                            'id_4x4': marker4['id'],
                            'id_5x5': marker5['id'],
                            'marker_coords_4x4': marker4['coords'],
                            'marker_coords_5x5': marker5['coords'],
                            'pos': list(self.current_target_pos)
                        }
                        self.collected.append(entry)
                        self.map_data.append(entry)
                        self.recorded_pairs.add(pair_id)

                        with open("map.json", "w") as f:
                            json.dump(self.map_data, f, indent=2)

                        print(f"[Mission] Записана пара: 4x4={marker4['id']}, "
                              f"5x5={marker5['id']} на {self.current_target_pos}")
                        return

    def get_results(self) -> List[Dict]:
        return self.collected
