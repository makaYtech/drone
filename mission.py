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

        # Размер одного квадрата и шаг спирали
        self.SQUARE_SIZE = 1.0
        self.SPIRAL_STEP = 0.1  # шаг спирали внутри квадрата
        
        # Калибровка
        self.CALIBRATION_DISTANCE = 10.0  # эталонное расстояние для калибровки

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

        # Калибровка
        self.calibration_time = 0.0
        self.calibration_start_time = 0.0
        self.effective_speed = 0.0

        # Спираль
        self.spiral_points = []
        self.current_spiral_idx = 0
        self.recorded_pairs = set()

        # Управление полётом по точкам
        self.target_point_sent = False
        self.command_sent_time = 0.0
        self.point_reached_time = 0.0
        self.current_target_pos = (0.0, 0.0)
        self.expected_flight_time = 0.0

        # Логика обхода квадратов
        self.current_square = (0, 0)  # (x_offset, y_offset) — смещение левого нижнего угла
        self.last_square_with_markers = (0, 0)
        self.direction = "forward"  # "forward" — наращиваем Y, "side" — наращиваем X
        self.squares_with_markers = []  # список квадратов, где нашли метки

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
            self._handle_move_to_square()
        elif self.state == self.STATE_RETURN_HOME:
            self._handle_return_home()

        return self.state

    def _send_point(self, x, y):
        """Отправляет команду на полёт и вычисляет ожидаемое время."""
        current_x, current_y = self.current_target_pos
        distance = math.sqrt((x - current_x)**2 + (y - current_y)**2)

        if self.effective_speed > 0:
            self.expected_flight_time = distance / self.effective_speed + 1.0
        else:
            self.expected_flight_time = 5.0

        self.drone.drone.go_to_local_point(x, y, 2, 0)
        self.target_point_sent = True
        self.command_sent_time = time.time()
        self.point_reached_time = 0.0
        self.current_target_pos = (x, y)
        print(f"[Mission] Отправлена команда: лететь в ({x:.2f}, {y:.2f}), "
              f"расстояние: {distance:.2f}м, ожидаемое время: {self.expected_flight_time:.2f}с")

    def _has_reached_point(self):
        """Проверка достижения точки через ожидаемое время полёта."""
        if not self.target_point_sent:
            return False
        return time.time() - self.command_sent_time >= self.expected_flight_time

    def _handle_waiting(self):
        if time.time() - self.state_start_time > 2.0:
            print("[Mission] Летим в точку (0, 0)...")
            self.state = self.STATE_GO_TO_ORIGIN
            self.current_target_pos = (0.0, 0.0)
            self._send_point(0, 0)

    def _handle_go_to_origin(self):
        if self._has_reached_point():
            print("[Mission] Прибыли в (0, 0). Начинаем калибровку...")
            self.state = self.STATE_CALIBRATION
            self.calibration_start_time = time.time()
            self._send_point(0, self.CALIBRATION_DISTANCE)

    def _handle_calibration(self):
        if self._has_reached_point():
            self.calibration_time = time.time() - self.calibration_start_time
            self.effective_speed = self.CALIBRATION_DISTANCE / self.calibration_time
            print(f"[Mission] Калибровка завершена. Время полёта {self.CALIBRATION_DISTANCE}м: "
                  f"{self.calibration_time:.2f}с, скорость: {self.effective_speed:.2f} м/с")
            self.state = self.STATE_RETURN_TO_ORIGIN
            self._send_point(0, 0)

    def _handle_return_to_origin(self):
        if self._has_reached_point():
            print("[Mission] Вернулись в (0, 0). Начинаем сканирование первого квадрата...")
            self.current_square = (0, 0)
            self._generate_spiral_points(self.current_square)
            self.state = self.STATE_SCAN_SQUARE
            self.current_spiral_idx = 0
            self.target_point_sent = False

    def _generate_spiral_points(self, square_offset):
        """Генерирует точки спирали для конкретного квадрата.
        square_offset = (x_off, y_off) — смещение левого нижнего угла.
        Спираль идёт от центра квадрата к краям, обязательно проходя через все 4 угла."""
        x_off, y_off = square_offset
        center = (x_off + self.SQUARE_SIZE / 2, y_off + self.SQUARE_SIZE / 2)
        
        self.spiral_points = []
        self.spiral_points.append(center)

        step = self.SPIRAL_STEP
        num_rings = int((self.SQUARE_SIZE / 2.0) / step)

        for ring in range(1, num_rings + 1):
            radius = ring * step
            x_min = x_off
            x_max = x_off + self.SQUARE_SIZE
            y_min = y_off
            y_max = y_off + self.SQUARE_SIZE

            # Ограничиваем рамкой текущего кольца
            x_min_ring = max(x_min, center[0] - radius)
            x_max_ring = min(x_max, center[0] + radius)
            y_min_ring = max(y_min, center[1] - radius)
            y_max_ring = min(y_max, center[1] + radius)

            # Верхняя сторона (слева направо)
            for x in np.arange(x_min_ring, x_max_ring + step / 2, step):
                self.spiral_points.append((round(x, 2), y_max_ring))
            # Правая сторона (сверху вниз)
            for y in np.arange(y_max_ring - step, y_min_ring - step / 2, -step):
                self.spiral_points.append((x_max_ring, round(y, 2)))
            # Нижняя сторона (справа налево)
            for x in np.arange(x_max_ring - step, x_min_ring - step / 2, -step):
                self.spiral_points.append((round(x, 2), y_min_ring))
            # Левая сторона (снизу вверх)
            for y in np.arange(y_min_ring + step, y_max_ring - step / 2, step):
                self.spiral_points.append((x_min_ring, round(y, 2)))

        # Убираем дубликаты
        unique_points = []
        seen = set()
        for point in self.spiral_points:
            rounded = (round(point[0], 2), round(point[1], 2))
            if rounded not in seen:
                seen.add(rounded)
                unique_points.append(point)
        self.spiral_points = unique_points

        # Гарантируем наличие углов текущего квадрата
        corners = [
            (x_off, y_off),
            (x_off, y_off + self.SQUARE_SIZE),
            (x_off + self.SQUARE_SIZE, y_off),
            (x_off + self.SQUARE_SIZE, y_off + self.SQUARE_SIZE)
        ]
        for corner in corners:
            rounded_corner = (round(corner[0], 2), round(corner[1], 2))
            if rounded_corner not in seen:
                self.spiral_points.append(corner)

        print(f"[Mission] Квадрат {square_offset}: сгенерировано {len(self.spiral_points)} точек спирали")

    def _handle_scan_square(self, markers: List[Dict]):
        """Сканирует текущий квадрат по спирали."""
        if not self.target_point_sent:
            if self.current_spiral_idx >= len(self.spiral_points):
                print(f"[Mission] Квадрат {self.current_square} просканирован.")
                self.state = self.STATE_DECIDE_NEXT
                return

            x, y = self.spiral_points[self.current_spiral_idx]
            print(f"[Mission] Квадрат {self.current_square}: летим к точке ({x:.2f}, {y:.2f}) "
                  f"[{self.current_spiral_idx}/{len(self.spiral_points)}]")
            self._send_point(x, y)
            return

        if self._has_reached_point():
            if self.point_reached_time == 0:
                self.point_reached_time = time.time()

            if time.time() - self.point_reached_time >= 1.0:
                self.current_spiral_idx += 1
                self.target_point_sent = False
                self.point_reached_time = 0

        self._check_and_record_markers(markers)

    def _handle_decide_next(self):
        """Решает, куда лететь дальше."""
        # Проверяем, нашли ли мы что-то в текущем квадрате
        found_in_current = any(
            self._is_in_square(entry['pos'], self.current_square)
            for entry in self.collected
        )
        
        if found_in_current:
            self.last_square_with_markers = self.current_square
            if self.current_square not in self.squares_with_markers:
                self.squares_with_markers.append(self.current_square)
            print(f"[Mission] В квадрате {self.current_square} найдены метки! "
                  f"Продолжаем в направлении '{self.direction}'.")

            # Продолжаем в том же направлении
            if self.direction == "forward":
                self.current_square = (self.current_square[0], self.current_square[1] + 1)
            else:  # side
                self.current_square = (self.current_square[0] + 1, self.current_square[1])
        else:
            # Не нашли меток в текущем квадрате
            print(f"[Mission] В квадрате {self.current_square} меток не найдено.")
            
            if self.direction == "forward":
                # Возвращаемся на последний квадрат с метками и переключаемся на "side"
                if self.last_square_with_markers == self.current_square:
                    # Мы ещё не нашли ни одного квадрата с метками — возвращаемся домой
                    print("[Mission] Меток не найдено ни в одном квадрате. Возвращаемся в (0, 0).")
                    self.state = self.STATE_RETURN_HOME
                    self._send_point(0, 0)
                    return
                
                print(f"[Mission] Возвращаемся на квадрат {self.last_square_with_markers} "
                      f"и переключаемся на боковой поиск.")
                self.current_square = (self.last_square_with_markers[0] + 1, 
                                      self.last_square_with_markers[1])
                self.direction = "side"
            else:  # direction == "side"
                # Боковой поиск тоже не дал результатов — возвращаемся домой
                print("[Mission] Боковой поиск не дал результатов. Возвращаемся в (0, 0).")
                self.state = self.STATE_RETURN_HOME
                self._send_point(0, 0)
                return
        
        print(f"[Mission] Переходим к квадрату {self.current_square}")
        self.state = self.STATE_MOVE_TO_SQUARE

    def _is_in_square(self, pos, square_offset):
        """Проверяет, находится ли позиция внутри квадрата."""
        x_off, y_off = square_offset
        x, y = pos[0], pos[1]
        return (x_off <= x <= x_off + self.SQUARE_SIZE and 
                y_off <= y <= y_off + self.SQUARE_SIZE)

    def _handle_move_to_square(self):
        """Летит в центр нового квадрата и начинает сканирование."""
        if not self.target_point_sent:
            x_off, y_off = self.current_square
            center_x = x_off + self.SQUARE_SIZE / 2
            center_y = y_off + self.SQUARE_SIZE / 2
            print(f"[Mission] Летим в центр квадрата {self.current_square}: ({center_x:.2f}, {center_y:.2f})")
            self._send_point(center_x, center_y)
            return

        if self._has_reached_point():
            print(f"[Mission] Прибыли в квадрат {self.current_square}. Начинаем сканирование.")
            self._generate_spiral_points(self.current_square)
            self.state = self.STATE_SCAN_SQUARE
            self.current_spiral_idx = 0
            self.target_point_sent = False

    def _handle_return_home(self):
        """Возврат в (0, 0) и завершение."""
        if self._has_reached_point():
            print(f"[Mission] Вернулись в (0, 0). Миссия завершена. "
                  f"Просканировано квадратов с метками: {len(self.squares_with_markers)}")
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

                        print(f"[Mission] Записана пара: 4x4={marker4['id']}, 5x5={marker5['id']} "
                              f"на позиции {self.current_target_pos}")
                        return

    def get_results(self) -> List[Dict]:
        return self.collected