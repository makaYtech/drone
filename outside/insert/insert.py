import math
import time
import json
from enum import Enum
from control.drone_position import StepCenteringController

class InsertState(Enum):
    INIT = 0
    FLY_TO_GLOBAL = 1
    SEARCHING = 2
    CENTERING_PIXELS = 3
    DESCEND_TO_1_5 = 4
    HOVER_AND_UPDATE = 5
    NEXT = 6
    RETURN_HOME = 7
    DONE = 8

class InsertMission:
    def __init__(self, drone_controller, map_file="map.json", frame_w=640, frame_h=480):
        self.drone = drone_controller
        self.map_file = map_file
        self.frame_w = frame_w
        self.frame_h = frame_h
        self.state = InsertState.INIT
        self.targets = []
        self.current_target_idx = 0

        # Пошаговый центровщик
        self.centering_ctrl = StepCenteringController(frame_w, frame_h)

        self.target_point_sent = False
        self.command_sent_time = 0.0
        self.hover_start_time = 0.0

        self.centering_start_time = 0.0
        self.marker_lost_time = 0.0
        self.marker_lost_duration = 2.0

        self.descend_speed = 0.08
        self.descend_start_time = 0.0

        self.search_phase = 0.0
        self.search_radius = 0.0

    def start(self):
        self.state = InsertState.INIT
        print("[Insert] Миссия запущена.")

    def _load_targets(self):
        try:
            with open(self.map_file, 'r') as f:
                self.targets = json.load(f)
                print(f"[Insert] Загружено {len(self.targets)} целей.")
                return True
        except Exception as e:
            print(f"[Insert] Ошибка загрузки: {e}")
            return False

    def _send_point(self, x, y, z=2.0, yaw=0.0):
        self.drone.go_to_point(x, y, z, yaw)
        self.target_point_sent = True
        self.command_sent_time = time.time()
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
            target = self.targets[self.current_target_idx]
            if not self.target_point_sent:
                self._send_point(target['pos'][0], target['pos'][1], z=2.0)
                return None
            if self._has_reached_point():
                print(f"[Insert] Прибыли в точку {self.current_target_idx+1}. Начинаем поиск маркера.")
                self.target_point_sent = False
                self.state = InsertState.SEARCHING
                self.search_phase = 0.0
            return None

        # ---- SEARCHING ----
        elif self.state == InsertState.SEARCHING:
            target = self.targets[self.current_target_idx]
            marker = self._find_target_marker(markers, target['id_4x4'])
            if marker:
                print(f"[Insert] ✅ Маркер найден! Начинаю центровку.")
                self.drone.set_manual_speed(0, 0, 0, 0)
                self.centering_ctrl.reset()
                self.centering_start_time = time.time()
                self.marker_lost_time = 0.0
                self.state = InsertState.CENTERING_PIXELS
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

        # ---- CENTERING_PIXELS ----
        elif self.state == InsertState.CENTERING_PIXELS:
            target = self.targets[self.current_target_idx]
            marker = self._find_target_marker(markers, target['id_4x4'])

            if not marker:
                if self.marker_lost_time == 0.0:
                    self.marker_lost_time = time.time()
                    print("[Insert] ⚠️ Маркер потерян, останавливаюсь...")
                    self.drone.set_manual_speed(0, 0, 0, 0)

                if time.time() - self.marker_lost_time > self.marker_lost_duration:
                    print("[Insert] Маркер не найден, перехожу в поиск.")
                    self.drone.set_manual_speed(0, 0, 0, 0)
                    self.state = InsertState.SEARCHING
                    self.marker_lost_time = 0.0
                    self.centering_ctrl.reset()
                return None

            self.marker_lost_time = 0.0
            vx, vy, vz, yaw_rate, centered_done = self.centering_ctrl.update(marker)
            self.drone.set_manual_speed(vx, vy, vz, yaw_rate)

            if centered_done:
                print("[Insert] 🎯 Центровка завершена! Опускаюсь.")
                self.drone.set_manual_speed(0, 0, 0, 0)
                self.state = InsertState.DESCEND_TO_1_5
                self.descend_start_time = time.time()
                return None

            if time.time() - self.centering_start_time > 45.0:
                print("[Insert] ⏱ Общий таймаут центрирования, принудительное опускание.")
                self.drone.set_manual_speed(0, 0, 0, 0)
                self.state = InsertState.DESCEND_TO_1_5
                self.descend_start_time = time.time()
            return None

        # ---- DESCEND_TO_1_5 ----
        elif self.state == InsertState.DESCEND_TO_1_5:
            target = self.targets[self.current_target_idx]
            marker = self._find_target_marker(markers, target['id_4x4'])
            if marker:
                mx, my = marker['center']
                cx = self.frame_w / 2
                cy = self.frame_h / 2
                err_x = mx - cx
                err_y = my - cy
                vy = 0.0
                vz = 0.0
                if abs(err_x) > 60:
                    vy = -self.centering_ctrl.sign_x * 0.01 if err_x > 0 else self.centering_ctrl.sign_x * 0.01
                if abs(err_y) > 60:
                    vz = -self.centering_ctrl.sign_y * 0.01 if err_y > 0 else self.centering_ctrl.sign_y * 0.01
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
            # Ждём 3 секунды с выводом сообщения о невозможности дальнейших действий
            if time.time() - self.hover_start_time < 3.0:
                # Выводим сообщение один раз
                if not hasattr(self, '_msg_printed'):
                    print("[Insert] ⚠️ Дальнейшие действия невозможны. Ожидание 3 секунды...")
                    self._msg_printed = True
                return None

            print("[Insert] Ожидание завершено.")
            self._msg_printed = False
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
