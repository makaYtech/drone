import time
from collections import deque
from typing import List, Dict

from constants import (
    DEFAULT_ALTITUDE, CALIBRATION_DISTANCE,
    WAIT_BEFORE_START
)
from outside.states import MissionStates
from outside.marker_handler import MarkerHandler
from outside.navigation import Navigator


class Mission:
    def __init__(self, drone_controller, initial_yaw: float = 0, target_groups: int = 4,
                 default_altitude: float = DEFAULT_ALTITUDE):
        self.drone = drone_controller
        self.target_groups = target_groups
        self.default_altitude = default_altitude
        self.CALIBRATION_DISTANCE = CALIBRATION_DISTANCE

        self.state = MissionStates.WAITING
        self.state_start_time = time.time()

        self.navigator = Navigator(drone_controller)
        self.marker_handler = MarkerHandler(square_size=5.0)

        self.grid_step = 2.5
        self.move_timeout = 10.0
        self.queue = deque()
        self.visited = set()
        self.current_square = None
        self.move_start_time = 0.0
        self.square_reached = False
        self._move_sent = False

        self.calibration_time = None
        self.movement_speed = 0.5

    def start(self):
        self.state = MissionStates.WAITING
        self.state_start_time = time.time()
        print("[Mission] Миссия запущена, ожидание 2 секунды...")

    def stop(self):
        """Принудительно завершает миссию (для перехода в Insert)."""
        if self.state not in (MissionStates.DONE, MissionStates.RETURN_HOME):
            print("[Mission] Принудительная остановка Exploration. Переходим в Insert.")
            self.state = MissionStates.DONE

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
        elif self.state == MissionStates.MOVE_TO_SQUARE:
            self._handle_move_to_square(markers)
        elif self.state == MissionStates.DECIDE_NEXT:
            self._handle_decide_next()
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
            print("[Mission] Вернулись в (0, 0). Начинаем BFS-обход.")
            self.queue.append((0, 0))
            self.visited.add((0, 0))
            self.state = MissionStates.DECIDE_NEXT
            self.navigator.reset_target()

    def _handle_move_to_square(self, markers: List[Dict]):
        if self.current_square is None:
            return

        if not self._move_sent:
            i, j = self.current_square
            center_x = i * self.grid_step + self.grid_step / 2
            center_y = j * self.grid_step + self.grid_step / 2

            pos = self.drone.get_position()
            if pos is not None:
                cur_x, cur_y, _ = pos
                dx = center_x - cur_x
                dy = center_y - cur_y
                dist = (dx**2 + dy**2)**0.5
            else:
                dist = self.grid_step

            self.move_timeout = max(8.0, min(30.0, 2.0 * dist / self.movement_speed + 3.0))
            print(f"[Mission] Летим в квадрат ({i}, {j}) -> центр ({center_x:.2f}, {center_y:.2f}), дистанция {dist:.1f} м, таймаут {self.move_timeout:.1f} сек")
            self.navigator.send_point(center_x, center_y, z=self.default_altitude)
            self.move_start_time = time.time()
            self.square_reached = False
            self._move_sent = True

        if self.navigator.has_reached_point():
            print(f"[Mission] Квадрат ({self.current_square[0]}, {self.current_square[1]}) достигнут.")
            self.square_reached = True
            pos = self.drone.get_position()
            if pos is not None:
                current_pos = (pos[0], pos[1])
            else:
                current_pos = self.navigator.current_target_pos
            self.marker_handler.check_and_record_markers(markers, current_pos)
            self.navigator.reset_target()
            self._move_sent = False
            self.state = MissionStates.DECIDE_NEXT
            return

        if time.time() - self.move_start_time > self.move_timeout:
            print(f"[Mission] Таймаут движения к квадрату ({self.current_square[0]}, {self.current_square[1]}), считаем стеной.")
            print("[Mission] Выполняем посадку и взлёт для сброса состояния...")
            self.drone.land()
            time.sleep(2)  # ждём, чтобы дрон успел приземлиться
            # Проверяем, что дрон действительно сел
            while self.drone.is_in_air():
                time.sleep(0.1)
            self.drone.takeoff()
            while not self.drone.is_in_air():
                time.sleep(0.1)
            self.navigator.reset_target()
            self._move_sent = False
            self.square_reached = False
            self.state = MissionStates.DECIDE_NEXT
            return

    def _handle_decide_next(self):
        if self.square_reached:
            i, j = self.current_square
            for di, dj in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                ni, nj = i + di, j + dj
                if (ni, nj) not in self.visited:
                    self.visited.add((ni, nj))
                    self.queue.append((ni, nj))

        if self.queue:
            self.current_square = self.queue.popleft()
            self.state = MissionStates.MOVE_TO_SQUARE
            self.navigator.reset_target()
            self._move_sent = False
            print(f"[Mission] Очередь: {len(self.queue)} квадратов. Следующий: {self.current_square}")
        else:
            print("[Mission] Все достижимые квадраты проверены. Возврат на базу.")
            self.state = MissionStates.RETURN_HOME
            self.navigator.send_point(0, 0, z=self.default_altitude)

    def _handle_return_home(self):
        if self.navigator.has_reached_point():
            print("[Mission] Возврат на базу выполнен. Миссия завершена.")
            self.state = MissionStates.DONE

    def get_results(self) -> List[Dict]:
        return self.marker_handler.get_results()
