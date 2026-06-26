import time
from typing import List, Dict
from constants import (
    SQUARE_SIZE, DEFAULT_ALTITUDE, CALIBRATION_DISTANCE,
    WAIT_BEFORE_START, TIMEOUT_POINT_REACHED
)
from outside.states import MissionStates
from outside.marker_handler import MarkerHandler
from outside.navigation import Navigator
from outside.path_planner import PathPlanner

class Mission:
    def __init__(self, drone_controller, initial_yaw: float = 0, target_groups: int = 4,
                 default_altitude: float = DEFAULT_ALTITUDE):
        self.drone = drone_controller
        self.target_groups = target_groups
        self.default_altitude = default_altitude
        self.CALIBRATION_DISTANCE = CALIBRATION_DISTANCE
        self.SQUARE_SIZE = SQUARE_SIZE

        self.state = MissionStates.WAITING
        self.state_start_time = time.time()

        self.navigator = Navigator(drone_controller)
        self.planner = PathPlanner(square_size=self.SQUARE_SIZE)
        self.marker_handler = MarkerHandler(square_size=self.SQUARE_SIZE)

        self.current_square = (0, 0)
        self.current_spiral_idx = 0
        self._target_spiral_idx = 0

        self.last_square_with_markers = (0, 0)
        self.direction = "forward"
        self.squares_with_markers = []
        self.has_been_backward = False
        self._after_return_action = None

        # Для калибровки
        self.calibration_time = None
        self.movement_speed = None
        self.straight_start_time = None
        self.straight_start_pos = (0.0, 0.0)
        self.straight_axis = None
        self.straight_direction = 1

    def start(self):
        self.state = MissionStates.WAITING
        self.state_start_time = time.time()
        print("[Mission] Миссия запущена, ожидание 2 секунды...")

    def update(self, markers: List[Dict]) -> str:
        if self.state == MissionStates.DONE:
            return "done"

        if self.state == MissionStates.WAITING:
            self._handle_waiting()
        elif self.state == MissionStates.GO_TO_ORIGIN:
            self._handle_go_to_origin()
        elif self.state == MissionStates.CALIBRATION:
            self._handle_calibration()
        elif self.state == MissionStates.RETURN_TO_ORIGIN:
            self._handle_return_to_origin()
        elif self.state == MissionStates.SCAN_SQUARE:
            self._handle_scan_square(markers)
        elif self.state == MissionStates.DECIDE_NEXT:
            self._handle_decide_next()
        elif self.state == MissionStates.MOVE_TO_SQUARE:
            self._handle_move_to_square(markers)
        elif self.state == MissionStates.RETURN_HOME:
            self._handle_return_home()

        return self.state

    def _handle_waiting(self):
        if time.time() - self.state_start_time > WAIT_BEFORE_START:
            print("[Mission] Летим в (0, 0)...")
            self.state = MissionStates.GO_TO_ORIGIN
            self.navigator.send_point(0, 0, z=self.default_altitude)

    def _handle_go_to_origin(self):
        if self.navigator.has_reached_point():
            print("[Mission] Прибыли в (0, 0). Начинаем калибровку...")
            self.state = MissionStates.CALIBRATION
            self.calibration_time = time.time()
            self.navigator.send_point(0, self.CALIBRATION_DISTANCE, z=self.default_altitude)

    def _handle_calibration(self):
        if self.navigator.has_reached_point():
            elapsed = time.time() - self.calibration_time
            if elapsed > 0:
                self.movement_speed = self.CALIBRATION_DISTANCE / elapsed
                print(f"[Mission] Скорость движения: {self.movement_speed:.3f} м/с (время {elapsed:.2f} сек)")
            else:
                self.movement_speed = 0.5
            print("[Mission] Возвращаемся в (0,0)...")
            self.state = MissionStates.RETURN_TO_ORIGIN
            self.navigator.send_point(0, 0, z=self.default_altitude)

    def _handle_return_to_origin(self):
        if self.navigator.has_reached_point():
            print("[Mission] Вернулись в (0, 0). Начинаем сканирование...")
            self.current_square = (0, 0)
            self.planner.generate_spiral_points(self.current_square)
            self.state = MissionStates.SCAN_SQUARE
            self.current_spiral_idx = 0
            self.navigator.reset_target()

    def _handle_scan_square(self, markers: List[Dict]):
        if not self.navigator.target_point_sent:
            if self.current_spiral_idx >= len(self.planner.spiral_points):
                print(f"[Mission] Квадрат {self.current_square} просканирован.")
                self.state = MissionStates.DECIDE_NEXT
                return

            line_end = self.planner.find_straight_line_end(self.current_spiral_idx)

            if line_end > self.current_spiral_idx:
                target = self.planner.spiral_points[line_end]
                print(f"[Mission] Прямая: {self.current_spiral_idx} → {line_end} к ({target[0]:.2f}, {target[1]:.2f})")
                self.navigator.send_point(target[0], target[1], z=self.default_altitude)
                self._target_spiral_idx = line_end

                start_pos = self.planner.spiral_points[self.current_spiral_idx]
                self.straight_start_pos = start_pos
                self.straight_start_time = time.time()
                dx = target[0] - start_pos[0]
                dy = target[1] - start_pos[1]
                if abs(dx) > 0.01:
                    self.straight_axis = 'x'
                    self.straight_direction = 1 if dx > 0 else -1
                else:
                    self.straight_axis = 'y'
                    self.straight_direction = 1 if dy > 0 else -1
            else:
                x, y = self.planner.spiral_points[self.current_spiral_idx]
                print(f"[Mission] Спираль: точка ({x:.2f}, {y:.2f}) [{self.current_spiral_idx}/{len(self.planner.spiral_points)}]")
                self.navigator.send_point(x, y, z=self.default_altitude)
                self._target_spiral_idx = self.current_spiral_idx
                self.straight_axis = None
            return

        # Определяем позицию для записи маркеров
        if self.straight_axis is not None and self.movement_speed is not None:
            elapsed = time.time() - self.straight_start_time
            distance = elapsed * self.movement_speed
            if self.straight_axis == 'x':
                real_x = self.straight_start_pos[0] + self.straight_direction * distance
                real_y = self.straight_start_pos[1]
            else:
                real_x = self.straight_start_pos[0]
                real_y = self.straight_start_pos[1] + self.straight_direction * distance
            marker_position = (real_x, real_y)
        else:
            marker_position = self.navigator.current_target_pos

        self.marker_handler.check_and_record_markers(markers, marker_position)

        if self.navigator.has_reached_point():
            self.navigator.mark_reached()

        if self.navigator.point_was_reached:
            if time.time() - self.navigator.point_reached_time >= 1.0:
                if hasattr(self, '_target_spiral_idx'):
                    self.current_spiral_idx = self._target_spiral_idx
                    del self._target_spiral_idx
                self.current_spiral_idx += 1
                self.navigator.reset_target()
                self.straight_axis = None

    def _handle_decide_next(self):
        found_in_current = any(
            self.marker_handler.is_in_square(entry['pos'], self.current_square)
            for entry in self.marker_handler.map_data
        )

        if found_in_current:
            self.last_square_with_markers = self.current_square
            if self.current_square not in self.squares_with_markers:
                self.squares_with_markers.append(self.current_square)
            print(f"[Mission] В квадрате {self.current_square} найдены метки! Продолжаем '{self.direction}'.")

            if self.direction == "forward":
                next_sq = (self.current_square[0], self.current_square[1] + self.SQUARE_SIZE)
            elif self.direction == "backward":
                next_sq = (self.current_square[0], self.current_square[1] - self.SQUARE_SIZE)
            else:  # side
                next_sq = (self.current_square[0] + self.SQUARE_SIZE, self.current_square[1])

            self.current_square = next_sq
            self.state = MissionStates.MOVE_TO_SQUARE
        else:
            print(f"[Mission] В квадрате {self.current_square} меток нет.")

            if self.direction == "forward":
                if len(self.squares_with_markers) == 0:
                    print("[Mission] Вперёд ничего. Пробуем назад.")
                    self.current_square = (0, -int(self.SQUARE_SIZE))
                    self.direction = "backward"
                    self.has_been_backward = True
                    self.state = MissionStates.MOVE_TO_SQUARE
                else:
                    print(f"[Mission] Возврат на {self.last_square_with_markers}, бок.")
                    self.current_square = (
                        self.last_square_with_markers[0] + int(self.SQUARE_SIZE),
                        self.last_square_with_markers[1])
                    self.direction = "side"
                    self.state = MissionStates.MOVE_TO_SQUARE
            elif self.direction == "backward":
                print(f"[Mission] Возврат на {self.last_square_with_markers}, бок.")
                self.current_square = (
                    self.last_square_with_markers[0] + int(self.SQUARE_SIZE),
                    self.last_square_with_markers[1])
                self.direction = "side"
                self.state = MissionStates.MOVE_TO_SQUARE
            else:  # side
                if not self.has_been_backward:
                    print("[Mission] Бок не дал. Возврат в (0,0), назад.")
                    self.state = MissionStates.RETURN_HOME
                    self.navigator.send_point(0, 0, z=self.default_altitude)
                    self._after_return_action = "backward"
                else:
                    print("[Mission] Всё проверено. Возврат в (0, 0).")
                    self.state = MissionStates.RETURN_HOME
                    self.navigator.send_point(0, 0, z=self.default_altitude)
                    self._after_return_action = "done"

    def _handle_move_to_square(self, markers: List[Dict]):
        if not self.navigator.target_point_sent:
            x_off, y_off = self.current_square
            center_x = x_off + self.SQUARE_SIZE / 2
            center_y = y_off + self.SQUARE_SIZE / 2
            print(f"[Mission] Летим в квадрат {self.current_square}: ({center_x:.2f}, {center_y:.2f})")
            self.navigator.send_point(center_x, center_y, z=self.default_altitude)
            return

        self.marker_handler.check_and_record_markers(markers, self.navigator.current_target_pos)

        if self.navigator.has_reached_point():
            print(f"[Mission] Прибыли в квадрат {self.current_square}. Спиральное сканирование.")
            self.planner.generate_spiral_points(self.current_square)
            self.state = MissionStates.SCAN_SQUARE
            self.current_spiral_idx = 0
            self.navigator.reset_target()

    def _handle_return_home(self):
        if self.navigator.has_reached_point():
            action = getattr(self, '_after_return_action', 'done')
            if action == "backward":
                print("[Mission] Вернулись в (0, 0). Движение назад.")
                self.current_square = (0, -int(self.SQUARE_SIZE))
                self.direction = "backward"
                self.has_been_backward = True
                self.planner.generate_spiral_points(self.current_square)
                self.state = MissionStates.SCAN_SQUARE
                self.current_spiral_idx = 0
                self.navigator.reset_target()
                self._after_return_action = None
            else:
                print(f"[Mission] Миссия завершена. Квадратов с метками: {len(self.squares_with_markers)}")
                self.state = MissionStates.DONE

    def get_results(self) -> List[Dict]:
        return self.marker_handler.get_results()
