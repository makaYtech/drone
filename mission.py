from enum import Enum
import time
import numpy as np
import json
import os

class MissionState(Enum):
    IDLE = 0
    TAKEOFF = 1
    SEARCH_FIRST = 2
    CENTERING = 3
    RECORD = 4
    MEASURE_DIST = 5
    NAVIGATE = 6
    ROTATE_SEARCH = 7
    RETURN_TO_LAST = 8
    COMPLETE = 9

class Mission:
    def __init__(self, drone_controller, initial_yaw=0, map_file="map.json", target_groups=4):
        self.drone = drone_controller
        self.map_file = map_file
        self.target_groups = target_groups
        self.initial_yaw = initial_yaw
        self.state = MissionState.IDLE

        self.results = []
        self.recorded_4x4_ids = set()
        self.map_data = []
        self.current_yaw = initial_yaw

        self.FORWARD_SPEED = 0.3
        self.SEARCH_TIMEOUT = 10.0
        self.GLOBAL_TIMEOUT = 120.0
        self.CENTER_TIMEOUT = 6.0
        self.CENTER_THRESHOLD = 0.05
        self.KP = 0.25
        self.KP_Z = 0.20
        self.TARGET_DISTANCE = 1.2
        self.MAX_SPEED = 0.3
        self.DEAD_ZONE = 0.02
        self.HOLD_DURATION = 2.0

        self.start_pos = (0.0, 0.0, 2.0)
        self.estimated_pos = self.start_pos
        self.last_move_time = None
        self.anchor_pos = None
        self.gap_distance = None
        self.temp_target = None

        self.target_point = None
        self.target_id = None
        self.center_start_time = None

        self.waypoints = []
        self.current_wp_idx = 0

        self.rotation_attempts = 0

        self.mission_start_time = None
        self.step_start_time = None
        self.hold_start_time = None
        self.group_counter = 0

    def start(self):
        if self.state == MissionState.IDLE:
            self.state = MissionState.TAKEOFF
            print("[Миссия] Взлёт...")

    def _update_odometry(self):
        if self.last_move_time is None:
            return
        dt = time.time() - self.last_move_time
        dy = self.FORWARD_SPEED * dt
        x, y, z = self.estimated_pos
        self.estimated_pos = (x, y + dy, z)
        self.last_move_time = time.time()

    def _go_to_point(self, x, y, z, yaw=None):
        if yaw is None:
            yaw = self.current_yaw
        self.drone.drone.go_to_local_point(x=x, y=y, z=z, yaw=yaw)
        print(f"[Миссия] Летим к точке ({x:.2f}, {y:.2f}, {z:.2f}) с курсом {np.degrees(yaw):.1f}°")

    def _wait_for_point_reached(self, timeout=10.0):
        start = time.time()
        while not self.drone.drone.point_reached():
            if time.time() - start > timeout:
                print("[Миссия] Таймаут ожидания достижения точки.")
                return False
            time.sleep(0.1)
        return True

    def _compute_center_velocities(self, target_point):
        if target_point is None:
            return (0, 0, 0, 0)
        x, y, z = target_point
        err_x = 0 if abs(x) < self.DEAD_ZONE else x
        err_z = 0 if abs(z - self.TARGET_DISTANCE) < self.DEAD_ZONE else (z - self.TARGET_DISTANCE)
        vx = -self.KP * err_x
        vy = self.KP_Z * err_z
        vz = 0.0
        vx = np.clip(vx, -self.MAX_SPEED, self.MAX_SPEED)
        vy = np.clip(vy, -self.MAX_SPEED, self.MAX_SPEED)
        return (float(vx), float(vy), 0.0, 0.0)

    def _is_centered(self, target_point):
        if target_point is None:
            return False
        x, y, z = target_point
        return (abs(x) < self.CENTER_THRESHOLD and
                abs(z - self.TARGET_DISTANCE) < self.CENTER_THRESHOLD)

    def _find_new_pair(self, markers_list):
        available_4x4 = [m for m in markers_list if m['type'] == '4x4' and m['id'] not in self.recorded_4x4_ids and m['coords'] is not None]
        available_5x5 = [m for m in markers_list if m['type'] == '5x5' and m['coords'] is not None]
        if available_4x4 and available_5x5:
            m4 = available_4x4[0]
            m5 = available_5x5[0]
            return m4['id'], m5['id'], m4['coords']
        return None

    def _find_pair_by_ids(self, markers_list):
        m4 = next((m for m in markers_list if m['type'] == '4x4' and m['id'] not in self.recorded_4x4_ids and m['coords'] is not None), None)
        m5 = next((m for m in markers_list if m['type'] == '5x5' and m['coords'] is not None), None)
        if m4 and m5:
            return m4['id'], m5['id'], m4['coords'], m5['coords']
        return None

    def _record_data(self, markers_list, pos):
        pair = self._find_pair_by_ids(markers_list)
        if pair:
            id4, id5, coords4, coords5 = pair
            entry = {
                'id_4x4': id4,
                'id_5x5': id5,
                'pos': pos,
                'marker_coords_4x4': coords4,
                'marker_coords_5x5': coords5
            }
            self.results.append(entry)
            self.recorded_4x4_ids.add(id4)
            self.group_counter += 1
            print(f"[Миссия] Записана группа {self.group_counter}: 4x4={id4}, 5x5={id5} на позиции {pos}")
            self.map_data.append({
                'id_4x4': id4,
                'id_5x5': id5,
                'pos': pos
            })
            self._save_map()
            return True
        return False

    def _save_map(self):
        with open(self.map_file, 'w') as f:
            json.dump(self.map_data, f, indent=4)

    def update(self, markers_list):
        if self.state == MissionState.IDLE:
            return 'idle'

        # ---- TAKEOFF ----
        elif self.state == MissionState.TAKEOFF:
            self.drone.arm()
            self.drone.takeoff()
            self.drone.drone.go_to_local_point(x=0, y=0, z=2, yaw=self.initial_yaw)
            self.last_move_time = time.time()
            self.mission_start_time = time.time()
            self.state = MissionState.SEARCH_FIRST
            self.step_start_time = time.time()
            print("[Миссия] Старт, движение вперёд для поиска первой пары.")
            self.drone.set_manual_speed(vx=0, vy=self.FORWARD_SPEED, vz=0, yaw_rate=0)
            return None

        # ---- SEARCH_FIRST ----
        elif self.state == MissionState.SEARCH_FIRST:
            if time.time() - self.mission_start_time > self.GLOBAL_TIMEOUT:
                print("[Миссия] Таймаут, завершаем.")
                self.state = MissionState.COMPLETE
                return 'done'

            pair = self._find_new_pair(markers_list)
            if pair:
                id4, id5, coords_4x4 = pair
                self.target_point = coords_4x4
                self.target_id = id4
                self.state = MissionState.CENTERING
                self.center_start_time = time.time()
                print(f"[Миссия] Найдена первая пара: 4x4={id4}, 5x5={id5}. Центрируюсь.")
                self.drone.hold_position()
                return None

            if time.time() - self.step_start_time > self.SEARCH_TIMEOUT:
                self._update_odometry()
                self.drone.set_manual_speed(vx=0, vy=self.FORWARD_SPEED, vz=0, yaw_rate=0)
                self.step_start_time = time.time()
            else:
                self.drone.set_manual_speed(vx=0, vy=self.FORWARD_SPEED, vz=0, yaw_rate=0)
            return None

        # ---- CENTERING ----
        elif self.state == MissionState.CENTERING:
            if self.target_point is None:
                print("[Миссия] target_point не установлен, переход к поиску.")
                self.state = MissionState.SEARCH_FIRST
                self.step_start_time = time.time()
                self.drone.set_manual_speed(vx=0, vy=self.FORWARD_SPEED, vz=0, yaw_rate=0)
                return None

            target_visible = any(
                m['type'] == '4x4' and m['id'] == self.target_id and m['coords'] is not None
                for m in markers_list
            )
            if not target_visible and self.target_point is not None:
                print("[Миссия] Метка пропала, переходим к записи.")
                self.state = MissionState.RECORD
                return None

            vx, vy, vz, yaw = self._compute_center_velocities(self.target_point)
            self.drone.set_manual_speed(vx, vy, vz, yaw)

            if self._is_centered(self.target_point):
                print("[Миссия] Центрирование достигнуто.")
                self.state = MissionState.RECORD
                return None

            if time.time() - self.center_start_time > self.CENTER_TIMEOUT:
                print("[Миссия] Таймаут центрирования.")
                self.state = MissionState.RECORD
                return None
            return None

        # ---- RECORD ----
        elif self.state == MissionState.RECORD:
            self._update_odometry()
            current_pos = self.estimated_pos

            if self._record_data(markers_list, current_pos):
                self.anchor_pos = current_pos
                if self.group_counter == 1:
                    self.state = MissionState.MEASURE_DIST
                    self.step_start_time = time.time()
                    self.last_move_time = time.time()
                    print("[Миссия] Первая группа записана, измеряем расстояние до второй.")
                    self.drone.set_manual_speed(vx=0, vy=self.FORWARD_SPEED, vz=0, yaw_rate=0)
                    return None
                else:
                    self.state = MissionState.NAVIGATE
                    self.step_start_time = time.time()
                    self.last_move_time = time.time()
                    print("[Миссия] Группа записана, продолжаем поиск.")
                    self.drone.set_manual_speed(vx=0, vy=self.FORWARD_SPEED, vz=0, yaw_rate=0)
                    return None
            else:
                self.state = MissionState.SEARCH_FIRST
                self.step_start_time = time.time()
                self.drone.set_manual_speed(vx=0, vy=self.FORWARD_SPEED, vz=0, yaw_rate=0)
                return None

        # ---- MEASURE_DIST ----
        elif self.state == MissionState.MEASURE_DIST:
            if self.anchor_pos is None:
                print("[Миссия] Ошибка: anchor_pos не установлен. Завершаем.")
                self.state = MissionState.COMPLETE
                return 'done'

            pair = self._find_new_pair(markers_list)
            if pair:
                id4, id5, coords_4x4 = pair
                self._update_odometry()
                dist = self.estimated_pos[1] - self.anchor_pos[1]
                self.gap_distance = dist
                print(f"[Миссия] Измерено расстояние между группами: {self.gap_distance:.2f} м")
                self.target_point = coords_4x4
                self.target_id = id4
                self.state = MissionState.CENTERING
                self.center_start_time = time.time()
                print(f"[Миссия] Найдена вторая пара: 4x4={id4}, 5x5={id5}. Центрируюсь.")
                self.drone.hold_position()
                return None

            if time.time() - self.step_start_time > self.SEARCH_TIMEOUT:
                self._update_odometry()
                self.drone.set_manual_speed(vx=0, vy=self.FORWARD_SPEED, vz=0, yaw_rate=0)
                self.step_start_time = time.time()
            else:
                self.drone.set_manual_speed(vx=0, vy=self.FORWARD_SPEED, vz=0, yaw_rate=0)
            return None

        # ---- NAVIGATE ----
        elif self.state == MissionState.NAVIGATE:
            if self.anchor_pos is None or self.gap_distance is None:
                print("[Миссия] Ошибка: anchor_pos или gap_distance не установлены. Завершаем.")
                self.state = MissionState.COMPLETE
                return 'done'

            self._update_odometry()
            dist_traveled = self.estimated_pos[1] - self.anchor_pos[1]

            if dist_traveled > self.gap_distance * 1.2:
                print("[Миссия] Пройдено расстояние, пара не найдена. Начинаем поиск с поворотами.")
                self.state = MissionState.ROTATE_SEARCH
                self.rotation_attempts = 0
                self.search_start_pos = self.estimated_pos
                self.state = MissionState.RETURN_TO_LAST
                return None

            pair = self._find_new_pair(markers_list)
            if pair:
                id4, id5, coords_4x4 = pair
                self.target_point = coords_4x4
                self.target_id = id4
                self.state = MissionState.CENTERING
                self.center_start_time = time.time()
                print(f"[Миссия] Найдена пара: 4x4={id4}, 5x5={id5}. Центрируюсь.")
                self.drone.hold_position()
                return None

            if time.time() - self.step_start_time > self.SEARCH_TIMEOUT:
                self.drone.set_manual_speed(vx=0, vy=self.FORWARD_SPEED, vz=0, yaw_rate=0)
                self.step_start_time = time.time()
            else:
                self.drone.set_manual_speed(vx=0, vy=self.FORWARD_SPEED, vz=0, yaw_rate=0)
            return None

        # ---- RETURN_TO_LAST ----
        elif self.state == MissionState.RETURN_TO_LAST:
            if self.anchor_pos is None:
                print("[Миссия] Ошибка: anchor_pos не установлен. Завершаем.")
                self.state = MissionState.COMPLETE
                return 'done'

            x, y, z = self.anchor_pos
            self._go_to_point(x, y, z)
            if self._wait_for_point_reached():
                print("[Миссия] Возврат на последнюю точку выполнен.")
                self.rotation_attempts += 1
                if self.rotation_attempts > 3:
                    print("[Миссия] Попытки исчерпаны, завершаем.")
                    self.state = MissionState.COMPLETE
                    return 'done'
                # Поворот
                if self.rotation_attempts == 1:
                    self.current_yaw += np.radians(90)
                elif self.rotation_attempts == 2:
                    self.current_yaw += np.radians(90)
                else:
                    self.state = MissionState.COMPLETE
                    return 'done'
                if self.gap_distance is None:
                    self.gap_distance = 2.0
                target_x = self.anchor_pos[0] + 2 * self.gap_distance * np.sin(self.current_yaw)
                target_y = self.anchor_pos[1] + 2 * self.gap_distance * np.cos(self.current_yaw)
                self.temp_target = (target_x, target_y, self.anchor_pos[2])
                print(f"[Миссия] Летим в новом направлении к точке {self.temp_target}")
                self._go_to_point(target_x, target_y, self.anchor_pos[2], yaw=self.current_yaw)
                if self._wait_for_point_reached():
                    self.anchor_pos = (target_x, target_y, self.anchor_pos[2])
                    self.state = MissionState.NAVIGATE
                    self.step_start_time = time.time()
                    self.last_move_time = time.time()
                    self.drone.set_manual_speed(vx=0, vy=self.FORWARD_SPEED, vz=0, yaw_rate=0)
                else:
                    self.state = MissionState.COMPLETE
                    return 'done'
            else:
                self.state = MissionState.COMPLETE
                return 'done'
            return None

        # ---- COMPLETE ----
        elif self.state == MissionState.COMPLETE:
            print("[Миссия] Завершена. Собрано групп:", len(self.results))
            return 'done'

        return None

    def get_results(self):
        return self.results
