import json
import time
import math
import numpy as np
from typing import List, Dict
from config import FORWARD_SPEED, YAW_SPEED

class Mission:
    def __init__(self, drone_controller, initial_yaw: float = 0, target_groups: int = 4):
        self.drone = drone_controller
        self.target_groups = target_groups
        self.collected = []
        self.map_data = []

        # Состояния
        self.STATE_WAITING = "waiting"
        self.STATE_SEARCH_FIRST = "search_first"
        self.STATE_APPROACHING = "approaching"
        self.STATE_RECORDING = "recording"
        self.STATE_MEASURE_GAP = "measure_gap"
        self.STATE_NAVIGATE = "navigate"
        self.STATE_RETURN_TO_LAST = "return_to_last"
        self.STATE_TURN_90 = "turn_90"
        self.STATE_RETURN_HOME = "return_home"
        self.STATE_DONE = "done"

        self.state = self.STATE_WAITING
        self.state_start_time = time.time()

        # Одометрия
        self.x = 0.0
        self.y = 0.0
        self.yaw = initial_yaw
        self.last_update_time = time.time()
        
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_yaw_rate = 0.0

        # Переменные миссии
        self.anchor_pos = (0.0, 0.0)
        self.gap_distance = 0.0
        self.distance_to_fly = 0.0
        self.rotation_attempts = 0
        
        self.found_4x4 = None
        self.found_5x5 = None
        self.target_marker_id = None

    def start(self):
        self.state = self.STATE_WAITING
        self.state_start_time = time.time()
        print("[Mission] Миссия запущена, ожидание 2 секунды...")

    def update(self, markers: List[Dict]) -> str:
        self._update_odometry()
        
        if self.state == self.STATE_DONE:
            return "done"

        if self.state == self.STATE_WAITING:
            self._handle_waiting()
        elif self.state == self.STATE_SEARCH_FIRST:
            self._handle_search_first(markers)
        elif self.state == self.STATE_APPROACHING:
            self._handle_approaching(markers)
        elif self.state == self.STATE_RECORDING:
            self._handle_recording(markers)
        elif self.state == self.STATE_MEASURE_GAP:
            self._handle_measure_gap(markers)
        elif self.state == self.STATE_NAVIGATE:
            self._handle_navigate(markers)
        elif self.state == self.STATE_RETURN_TO_LAST:
            self._handle_return_to_last()
        elif self.state == self.STATE_TURN_90:
            self._handle_turn_90()
        elif self.state == self.STATE_RETURN_HOME:
            self._handle_return_home()

        return self.state

    def _update_odometry(self):
        now = time.time()
        dt = now - self.last_update_time
        self.last_update_time = now
        if dt > 0.5: dt = 0.5
        
        # vy - вперёд, vx - влево. yaw=0 это ось +Y
        dx = (self.current_vy * math.sin(self.yaw) - self.current_vx * math.cos(self.yaw)) * dt
        dy = (self.current_vy * math.cos(self.yaw) + self.current_vx * math.sin(self.yaw)) * dt
        
        self.x += dx
        self.y += dy
        self.yaw += self.current_yaw_rate * dt
        self.yaw = (self.yaw + math.pi) % (2 * math.pi) - math.pi

    def _set_speed(self, vx, vy, vz, yaw_rate):
        self.current_vx = vx
        self.current_vy = vy
        self.current_yaw_rate = yaw_rate
        self.drone.set_manual_speed(vx, vy, vz, yaw_rate)
    
    def _stop(self):
        self._set_speed(0, 0, 0, 0)

    def _check_inertia_stop(self):
        """НЕБЛОКИРУЮЩЕЕ гашение инерции. Возвращает True, когда остановка завершена."""
        if not hasattr(self, '_inertia_end_time'):
            self._inertia_end_time = time.time() + 0.4
            self._set_speed(0, -FORWARD_SPEED * 0.5, 0, 0)  # vy — назад
            return False
        
        if time.time() >= self._inertia_end_time:
            self._stop()
            del self._inertia_end_time
            return True
            
        self._set_speed(0, -FORWARD_SPEED * 0.5, 0, 0)
        return False

    def _find_pair(self, markers):
        m4 = [m for m in markers if m['type'] == '4x4' and m['coords'] is not None]
        m5 = [m for m in markers if m['type'] == '5x5' and m['coords'] is not None]
        if m4 and m5:
            for marker4 in m4:
                for marker5 in m5:
                    if not any(r['id_4x4'] == marker4['id'] for r in self.collected):
                        return marker4, marker5
        return None, None

    def _handle_waiting(self):
        if time.time() - self.state_start_time > 2.0:
            print("[Mission] Начинаем прямой поиск первой пары...")
            self.state = self.STATE_SEARCH_FIRST
            self.state_start_time = time.time()
            self._set_speed(0, FORWARD_SPEED, 0, 0)

    def _handle_search_first(self, markers):
        m4, m5 = self._find_pair(markers)
        if m4 and m5:
            print(f"[Mission] Найдена первая пара! 4x4={m4['id']}, 5x5={m5['id']}")
            self.found_4x4 = m4
            self.found_5x5 = m5
            self.target_marker_id = m4['id']
            # Гасим инерцию неблокирующе
            if self._check_inertia_stop():
                self.state = self.STATE_APPROACHING
                self.state_start_time = time.time()
            return
            
        self._set_speed(0, FORWARD_SPEED, 0, 0)

    def _handle_approaching(self, markers):
        target = None
        for m in markers:
            if m['type'] == '4x4' and m['id'] == self.target_marker_id and m['coords'] is not None:
                target = m
                break
                
        if target is None:
            if time.time() - self.state_start_time > 3.0:
                print("[Mission] Маркер потерян при подлёте, переход к записи.")
                if self._check_inertia_stop():
                    self.state = self.STATE_RECORDING
            else:
                self._stop()
            return

        coords = target['coords']
        target_distance = 0.8
        kp_xy = 0.4
        kp_z = 0.5
        
        err_x = coords[0]
        err_y = coords[1]
        err_z = coords[2] - target_distance
        
        # ИСПРАВЛЕНИЕ 1: Добавляем мёртвые зоны, чтобы дрон не "ёлся" и не улетал назад
        if abs(err_z) < 0.05:
            vy = 0.0
        else:
            vy = kp_z * err_z
            
        if abs(err_x) < 0.03:
            vx = 0.0
        else:
            vx = -kp_xy * err_x
        
        vx = np.clip(vx, -FORWARD_SPEED, FORWARD_SPEED)
        vy = np.clip(vy, -FORWARD_SPEED, FORWARD_SPEED)
        
        self._set_speed(vx, vy, 0, 0)
        
        # ИСПРАВЛЕНИЕ 2: Гасим инерцию перед переходом в RECORD
        if abs(err_x) < 0.05 and abs(err_y) < 0.05 and abs(err_z) < 0.1:
            if self._check_inertia_stop():
                print("[Mission] Подлёт завершён и инерция погашена!")
                self.state = self.STATE_RECORDING
                self.state_start_time = time.time()

    def _handle_recording(self, markers):
        entry = {
            "id_4x4": self.found_4x4['id'],
            "id_5x5": self.found_5x5['id'],
            "pos": [round(self.x, 2), round(self.y, 2)]
        }
        self.collected.append(entry)
        self.map_data.append(entry)

        with open("map.json", "w") as f:
            json.dump(self.map_data, f, indent=2)

        print(f"[Mission] Записана пара #{len(self.collected)}: {entry}")
        
        self.anchor_pos = (self.x, self.y)

        if len(self.collected) == 1:
            self.state = self.STATE_MEASURE_GAP
            print("[Mission] Ищем вторую пару для измерения gap...")
        elif len(self.collected) >= self.target_groups:
            print("[Mission] Все пары собраны! Возвращаемся домой.")
            self.state = self.STATE_RETURN_HOME
        else:
            self.state = self.STATE_NAVIGATE
            self.distance_to_fly = self.gap_distance * 1.5
            print(f"[Mission] Летим на {self.distance_to_fly:.2f} м...")
            
        self.state_start_time = time.time()
        self._set_speed(0, FORWARD_SPEED, 0, 0)

    def _handle_measure_gap(self, markers):
        m4, m5 = self._find_pair(markers)
        if m4 and m5:
            dist = math.hypot(self.x - self.anchor_pos[0], self.y - self.anchor_pos[1])
            self.gap_distance = dist
            print(f"[Mission] Измерен gap: {self.gap_distance:.2f} м")
            
            self.found_4x4 = m4
            self.found_5x5 = m5
            self.target_marker_id = m4['id']
            
            if self._check_inertia_stop():
                self.state = self.STATE_APPROACHING
                self.state_start_time = time.time()
            return
            
        self._set_speed(0, FORWARD_SPEED, 0, 0)

    def _handle_navigate(self, markers):
        m4, m5 = self._find_pair(markers)
        if m4 and m5:
            print(f"[Mission] Найдена пара в навигации! 4x4={m4['id']}, 5x5={m5['id']}")
            self.found_4x4 = m4
            self.found_5x5 = m5
            self.target_marker_id = m4['id']
            if self._check_inertia_stop():
                self.state = self.STATE_APPROACHING
                self.state_start_time = time.time()
            return

        dist_traveled = math.hypot(self.x - self.anchor_pos[0], self.y - self.anchor_pos[1])
        if dist_traveled >= self.distance_to_fly:
            print("[Mission] Пролетели gap*1.5, пара не найдена. Возвращаемся.")
            if self._check_inertia_stop():
                self.state = self.STATE_RETURN_TO_LAST
                self.state_start_time = time.time()
            return
            
        self._set_speed(0, FORWARD_SPEED, 0, 0)

    def _handle_return_to_last(self):
        target_x, target_y = self.anchor_pos
        dx = target_x - self.x
        dy = target_y - self.y
        dist = math.hypot(dx, dy)

        # ИСПРАВЛЕНИЕ 3: Убран бесконечный цикл. Используем неблокирующий _check_inertia_stop
        if dist < 0.15:
            if self._check_inertia_stop():
                print("[Mission] Остановились на последней точке. Поворачиваем на 90°.")
                self.state = self.STATE_TURN_90
                self.state_start_time = time.time()
                self.rotation_attempts += 1
            return

        desired_yaw = math.atan2(dx, dy)
        yaw_err = (desired_yaw - self.yaw + math.pi) % (2 * math.pi) - math.pi

        if abs(yaw_err) > 0.15:
            yaw_rate = np.clip(yaw_err * 1.5, -YAW_SPEED, YAW_SPEED)
            self._set_speed(0, 0, 0, yaw_rate)
        else:
            self._set_speed(0, FORWARD_SPEED, 0, 0)

    def _handle_turn_90(self):
        if not hasattr(self, '_turn_target_yaw'):
            # ИСПРАВЛЕНИЕ 4: Добавляем +0.1 рад (~5.7°) для компенсации недоворота
            self._turn_target_yaw = self.yaw + math.pi / 2 + 0.1
            
        yaw_err = (self._turn_target_yaw - self.yaw + math.pi) % (2 * math.pi) - math.pi
        
        # Ужесточаем допуск с 0.15 до 0.05 и увеличиваем коэффициент
        if abs(yaw_err) > 0.05:
            yaw_rate = np.clip(yaw_err * 2.5, -YAW_SPEED, YAW_SPEED)
            self._set_speed(0, 0, 0, yaw_rate)
        else:
            self._stop()
            del self._turn_target_yaw
            
            if self.rotation_attempts >= 4:
                print("[Mission] Все направления проверены. Возвращаемся домой.")
                self.state = self.STATE_RETURN_HOME
            else:
                print(f"[Mission] Летим в новом направлении (попытка {self.rotation_attempts}).")
                self.state = self.STATE_NAVIGATE
                self.distance_to_fly = self.gap_distance * 1.5
                self.anchor_pos = (self.x, self.y)
                
            self.state_start_time = time.time()

    def _handle_return_home(self):
        target_x, target_y = 0.0, 0.0
        dx = target_x - self.x
        dy = target_y - self.y
        dist = math.hypot(dx, dy)
        
        if dist < 0.2:
            if self._check_inertia_stop():
                print("[Mission] Вернулись на точку старта (0,0). Завершаем.")
                self.state = self.STATE_DONE
            return
            
        desired_yaw = math.atan2(dx, dy)
        yaw_err = (desired_yaw - self.yaw + math.pi) % (2 * math.pi) - math.pi
        
        if abs(yaw_err) > 0.15:
            yaw_rate = np.clip(yaw_err * 1.5, -YAW_SPEED, YAW_SPEED)
            self._set_speed(0, 0, 0, yaw_rate)
        else:
            self._set_speed(0, FORWARD_SPEED, 0, 0)

    def get_results(self) -> List[Dict]:
        return self.collected
