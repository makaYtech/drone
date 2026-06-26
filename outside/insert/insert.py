import math
import time
import json
from enum import Enum

class InsertState(Enum):
    INIT = 0
    FLY_TO_GLOBAL = 1
    SEARCHING = 2
    CENTERING_HORIZONTAL = 3
    CENTERING_VERTICAL = 4
    DESCEND_TO_1_5 = 5
    HOVER_AND_UPDATE = 6
    NEXT = 7
    RETURN_HOME = 8
    DONE = 9

class SimpleCenteringLogic:
    def __init__(self, frame_w=640, frame_h=480):
        self.frame_w = frame_w
        self.frame_h = frame_h
        self.dead_zone_x = 100.0
        self.dead_zone_y = 60.0
        self.speed_x = 0.08
        self.speed_y = 0.04
        # Исправляем оси: обе оси с положительным знаком (если маркер правее – летим вправо, если ниже – летим вниз)
        self.sign_x = 1.0
        self.sign_y = 1.0

    def get_error(self, marker):
        if marker is None or 'center' not in marker:
            return None
        mx, my = marker['center']
        cx = self.frame_w / 2
        cy = self.frame_h / 2
        return (mx - cx, my - cy)

    def is_centered_x(self, err_x):
        return abs(err_x) < self.dead_zone_x

    def is_centered_y(self, err_y):
        return abs(err_y) < self.dead_zone_y

    def compute_speed_x(self, err_x):
        if abs(err_x) <= self.dead_zone_x:
            return 0.0
        return -self.sign_x * self.speed_x if err_x > 0 else self.sign_x * self.speed_x

    def compute_speed_y(self, err_y):
        if abs(err_y) <= self.dead_zone_y:
            return 0.0
        return -self.sign_y * self.speed_y if err_y > 0 else self.sign_y * self.speed_y

class InsertMission:
    def __init__(self, drone_controller, map_file="map.json", frame_w=640, frame_h=480):
        self.drone = drone_controller
        self.map_file = map_file
        self.frame_w = frame_w
        self.frame_h = frame_h
        self.state = InsertState.INIT
        self.targets = []
        self.current_target_idx = 0

        self.centering_ctrl = SimpleCenteringLogic(frame_w, frame_h)
        # Если после исправления оси всё равно инвертированы – раскомментируйте:
        # self.centering_ctrl.sign_y = -1.0

        self.target_point_sent = False
        self.command_sent_time = 0.0
        self.hover_start_time = 0.0

        self.centering_start_time = 0.0
        self.marker_lost_time = 0.0
        self.marker_lost_duration = 3.0

        self.descend_speed = 0.08
        self.descend_start_time = 0.0

        self.search_phase = 0.0
        self.search_timeout = 10.0
        self.search_start_time = 0.0

        self.current_global_pos = (0.0, 0.0)
        self.stable_frames = 0
        self.required_stable_frames = 15
        self.debug_counter = 0

        self.last_speeds = (0.0, 0.0, 0.0, 0.0)
        self.marker_was_seen = False

    def start(self):
        self.state = InsertState.INIT
        print("[Insert] Миссия запущена (раздельная центровка).")

    def _load_targets(self):
        try:
            with open(self.map_file, 'r') as f:
                self.targets = json.load(f)
                print(f"[Insert] Загружено {len(self.targets)} целей.")
                return True
        except Exception as e:
            print(f"[Insert] Ошибка загрузки: {e}")
            return False

    def _save_targets(self):
        with open(self.map_file, 'w') as f:
            json.dump(self.targets, f, indent=2)
        print("[Insert] Карта обновлена.")

    def _remove_current_target(self):
        """Удаляет текущую цель и сохраняет карту. Индекс не меняется."""
        if self.current_target_idx < len(self.targets):
            removed = self.targets.pop(self.current_target_idx)
            self._save_targets()
            print(f"[Insert] Цель {removed} удалена (маркер не найден).")
            # После удаления индекс остаётся тем же, т.к. элементы сдвинулись
            return True
        else:
            print("[Insert] Ошибка: индекс текущей цели вне диапазона.")
            return False

    def _update_current_target_position(self):
        if self.current_target_idx < len(self.targets):
            x, y = self.current_global_pos
            self.targets[self.current_target_idx]['pos'] = [x, y]
            self._save_targets()
            print(f"[Insert] Обновлены координаты цели: ({x:.2f}, {y:.2f})")
        else:
            print("[Insert] Ошибка: индекс текущей цели вне диапазона.")

    def _send_point(self, x, y, z=2.0, yaw=0.0):
        self.drone.go_to_point(x, y, z, yaw)
        self.target_point_sent = True
        self.command_sent_time = time.time()
        self.current_global_pos = (x, y)
        print(f"[Insert] Летим в ({x:.2f}, {y:.2f}, {z:.2f})")

    def _has_reached_point(self):
        if not self.target_point_sent:
            return False
        if time.time() - self.command_sent_time < 2.0:
            return False
        return self.drone.point_reached()

    def _find_target_marker(self, markers, target_id_4x4):
        for m in markers:
            if m['type'] == '4x4' and int(m['id']) == int(target_id_4x4):
                return m
        return None

    def update(self, markers):
        if self.state == InsertState.DONE:
            return "done"

        # ---- INIT ----
        if self.state == InsertState.INIT:
            if self._load_targets() and len(self.targets) > 0:
                self.current_target_idx = 0
                self.state = InsertState.FLY_TO_GLOBAL
            else:
                self.state = InsertState.DONE
            return None

        # ---- FLY_TO_GLOBAL ----
        elif self.state == InsertState.FLY_TO_GLOBAL:
            if self.current_target_idx >= len(self.targets):
                # Все цели обработаны (или удалены)
                self.state = InsertState.RETURN_HOME
                return None

            target = self.targets[self.current_target_idx]
            if not self.target_point_sent:
                self._send_point(target['pos'][0], target['pos'][1], z=2.0)
                return None
            if self._has_reached_point():
                print(f"[Insert] Прибыли в точку {self.current_target_idx+1}. Начинаем поиск маркера.")
                self.target_point_sent = False
                self.state = InsertState.SEARCHING
                self.search_phase = 0.0
                self.search_start_time = time.time()
            return None

        # ---- SEARCHING ----
        elif self.state == InsertState.SEARCHING:
            if time.time() - self.search_start_time > self.search_timeout:
                print(f"[Insert] ⏱ Время поиска истекло (>{self.search_timeout} сек). Удаляем цель.")
                removed = self._remove_current_target()
                if not removed:
                    # Если не удалось удалить (ошибка), переходим к следующей
                    self.state = InsertState.NEXT
                    return None

                # После удаления проверяем, остались ли цели
                if len(self.targets) == 0:
                    print("[Insert] Все цели удалены. Возврат на базу.")
                    self.state = InsertState.RETURN_HOME
                    return None
                else:
                    # Переходим к следующей цели (индекс не меняется, так как удалённый элемент заменился следующим)
                    self.state = InsertState.FLY_TO_GLOBAL
                    self.target_point_sent = False
                return None

            target = self.targets[self.current_target_idx]
            marker = self._find_target_marker(markers, target['id_4x4'])
            if marker:
                print(f"[Insert] ✅ Маркер найден! Начинаю центровку (горизонталь).")
                self.drone.set_manual_speed(0, 0, 0, 0)
                self.centering_start_time = time.time()
                self.marker_lost_time = 0.0
                self.stable_frames = 0
                self.debug_counter = 0
                self.marker_was_seen = True
                self.last_speeds = (0.0, 0.0, 0.0, 0.0)
                self.state = InsertState.CENTERING_HORIZONTAL
                return None

            self.search_phase += 0.04
            phase = self.search_phase % 10.0
            if phase < 4.0:
                self.drone.set_manual_speed(0, 0, 0, 0.25)
            elif phase < 8.0:
                self.drone.set_manual_speed(0, 0, 0, -0.25)
            else:
                self.drone.set_manual_speed(0, 0, 0, 0)
            return None

        # ---- CENTERING_HORIZONTAL ----
        elif self.state == InsertState.CENTERING_HORIZONTAL:
            target = self.targets[self.current_target_idx]
            marker = self._find_target_marker(markers, target['id_4x4'])

            if marker:
                self.marker_was_seen = True
                self.marker_lost_time = 0.0
                err_x, err_y = self.centering_ctrl.get_error(marker)
                vy = self.centering_ctrl.compute_speed_x(err_x)
                vz = 0.0
                self.last_speeds = (0.0, vy, vz, 0.0)
            else:
                if self.marker_was_seen:
                    if self.marker_lost_time == 0.0:
                        self.marker_lost_time = time.time()
                else:
                    self.drone.set_manual_speed(0, 0, 0, 0)
                    return None

                if self.marker_lost_time > 0 and time.time() - self.marker_lost_time > self.marker_lost_duration:
                    print("[Insert] Маркер не появляется, перехожу в поиск.")
                    self.drone.set_manual_speed(0, 0, 0, 0)
                    self.state = InsertState.SEARCHING
                    self.marker_lost_time = 0.0
                    self.marker_was_seen = False
                    self.search_start_time = time.time()
                    return None

            vx, vy, vz, yaw = self.last_speeds
            self.drone.set_manual_speed(vx, vy, vz, yaw)

            self.debug_counter += 1
            if self.debug_counter % 10 == 0 and marker:
                err_x, _ = self.centering_ctrl.get_error(marker)
                print(f"[Insert] Гор. err_x={err_x:.1f}px, vy={vy:.3f}")

            if marker:
                err_x, err_y = self.centering_ctrl.get_error(marker)
                if self.centering_ctrl.is_centered_x(err_x):
                    print("[Insert] Горизонталь отцентрирована! Перехожу к вертикали.")
                    self.drone.set_manual_speed(0, 0, 0, 0)
                    self.stable_frames = 0
                    self.debug_counter = 0
                    self.state = InsertState.CENTERING_VERTICAL
                    return None

            if time.time() - self.centering_start_time > 120.0:
                print("[Insert] ⏱ Таймаут центрирования, принудительное опускание.")
                self.drone.set_manual_speed(0, 0, 0, 0)
                self.state = InsertState.DESCEND_TO_1_5
                self.descend_start_time = time.time()
            return None

        # ---- CENTERING_VERTICAL ----
        elif self.state == InsertState.CENTERING_VERTICAL:
            target = self.targets[self.current_target_idx]
            marker = self._find_target_marker(markers, target['id_4x4'])

            if marker:
                self.marker_was_seen = True
                self.marker_lost_time = 0.0
                err_x, err_y = self.centering_ctrl.get_error(marker)
                vz = self.centering_ctrl.compute_speed_y(err_y)
                vy = 0.0
                self.last_speeds = (0.0, vy, vz, 0.0)
            else:
                if self.marker_was_seen:
                    if self.marker_lost_time == 0.0:
                        self.marker_lost_time = time.time()
                else:
                    self.drone.set_manual_speed(0, 0, 0, 0)
                    return None

                if self.marker_lost_time > 0 and time.time() - self.marker_lost_time > self.marker_lost_duration:
                    print("[Insert] Маркер не появляется, перехожу в поиск.")
                    self.drone.set_manual_speed(0, 0, 0, 0)
                    self.state = InsertState.SEARCHING
                    self.marker_lost_time = 0.0
                    self.marker_was_seen = False
                    self.search_start_time = time.time()
                    return None

            vx, vy, vz, yaw = self.last_speeds
            self.drone.set_manual_speed(vx, vy, vz, yaw)

            self.debug_counter += 1
            if self.debug_counter % 10 == 0 and marker:
                _, err_y = self.centering_ctrl.get_error(marker)
                print(f"[Insert] Верт. err_y={err_y:.1f}px, vz={vz:.3f}")

            if marker:
                _, err_y = self.centering_ctrl.get_error(marker)
                if self.centering_ctrl.is_centered_y(err_y):
                    self.stable_frames += 1
                    if self.stable_frames >= self.required_stable_frames:
                        print("[Insert] 🎯 Вертикаль отцентрирована! Опускаюсь.")
                        self.drone.set_manual_speed(0, 0, 0, 0)
                        self.state = InsertState.DESCEND_TO_1_5
                        self.descend_start_time = time.time()
                        return None
                else:
                    self.stable_frames = 0

            if time.time() - self.centering_start_time > 120.0:
                print("[Insert] ⏱ Таймаут центрирования, принудительное опускание.")
                self.drone.set_manual_speed(0, 0, 0, 0)
                self.state = InsertState.DESCEND_TO_1_5
                self.descend_start_time = time.time()
            return None

        # ---- DESCEND_TO_1_5 ----
        elif self.state == InsertState.DESCEND_TO_1_5:
            target = self.targets[self.current_target_idx]
            marker = self._find_target_marker(markers, target['id_4x4'])
            if marker:
                err_x, err_y = self.centering_ctrl.get_error(marker)
                vy = 0.0
                vz = 0.0
                if abs(err_x) > 60:
                    vy = -self.centering_ctrl.sign_x * 0.02 if err_x > 0 else self.centering_ctrl.sign_x * 0.02
                if abs(err_y) > 60:
                    vz = -self.centering_ctrl.sign_y * 0.02 if err_y > 0 else self.centering_ctrl.sign_y * 0.02
                self.drone.set_manual_speed(0, vy, -self.descend_speed, 0)
            else:
                self.drone.set_manual_speed(0, 0, -self.descend_speed, 0.1)

            if time.time() - self.descend_start_time > 6.0:
                print("[Insert] 📉 Опустился до ~1.5 м. Ожидаю 3 секунды.")
                self.drone.set_manual_speed(0, 0, 0, 0)
                self.state = InsertState.HOVER_AND_UPDATE
                self.hover_start_time = time.time()
            return None

        # ---- HOVER_AND_UPDATE ----
        elif self.state == InsertState.HOVER_AND_UPDATE:
            if time.time() - self.hover_start_time < 3.0:
                if not hasattr(self, '_msg_printed'):
                    print("[Insert] ⚠️ Дальнейшие действия невозможны. Ожидание 3 секунды...")
                    self._msg_printed = True
                return None

            print("[Insert] Ожидание завершено. Обновляем координаты цели.")
            self._msg_printed = False
            self._update_current_target_position()
            self.state = InsertState.NEXT
            return None

        # ---- NEXT ----
        elif self.state == InsertState.NEXT:
            self.current_target_idx += 1
            if self.current_target_idx < len(self.targets):
                self.state = InsertState.FLY_TO_GLOBAL
                self.target_point_sent = False
            else:
                print("[Insert] Все цели обработаны. Возврат на базу.")
                self._send_point(0, 0, 2.0)
                self.state = InsertState.RETURN_HOME
            return None

        # ---- RETURN_HOME ----
        elif self.state == InsertState.RETURN_HOME:
            if self._has_reached_point():
                print("[Insert] Возврат на базу выполнен. Миссия завершена.")
                self.state = InsertState.DONE
            return None

        # ---- DONE ----
        elif self.state == InsertState.DONE:
            return "done"

        return None

    def get_status(self):
        return "done" if self.state == InsertState.DONE else "running"