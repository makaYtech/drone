import time
from enum import Enum

from constants import (
    TIMEOUT_SEARCH, TIMEOUT_DESCEND, TIMEOUT_MARKER_LOST,
    TIMEOUT_MARKER_LOST_MOVEMENT,
    CENTERING_PAUSE_BEFORE, CENTERING_PAUSE_AFTER,
    CENTERING_MAX_ATTEMPTS, CENTERING_DESCENT_ALTITUDE,
    DEFAULT_ALTITUDE, STABILIZE_TIME, HEIGHT_STEP,
    DESCENT_TIMEOUT
)
from outside.centering_controller import CenteringController

class InsertState(Enum):
    INIT = 0
    FLY_TO_GLOBAL = 1
    STABILIZE = 2
    SEARCHING = 3
    PRE_CENTERING_PAUSE = 4
    CENTERING_NEXT_MOVE = 5
    CENTERING_WAITING = 6
    CENTERING_ANALYZE = 7
    WAITING_FOR_MARKER = 8
    ROLLBACK = 9
    CENTERED_PAUSE = 10
    UPDATE_MAP = 11
    NEXT = 12
    RETURN_HOME = 13
    DONE = 14
    POST_CENTERING_PAUSE = 15


class InsertMission:
    def __init__(self, drone_controller, marker_handler, frame_w=640, frame_h=480):
        self.drone = drone_controller
        self.marker_handler = marker_handler
        self.frame_w = frame_w
        self.frame_h = frame_h
        self.state = InsertState.INIT
        self.targets = []
        self.current_target_idx = 0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = DEFAULT_ALTITUDE

        self.centering_start_x = 0.0
        self.centering_start_y = 0.0

        self._move_sent = False
        self._target_reached = False
        self._target_reached_time = 0.0
        self._command_sent_time = 0.0

        self.centering_start_time = 0.0
        self.search_start_time = 0.0
        self.waiting_start_time = 0.0
        self.lost_during_movement = False
        self.descent_timeout = DESCENT_TIMEOUT
        self._msg_printed = False

        # Мигание зелёным (только для первой цели)
        self._blink_active = False
        self._blink_start_time = 0.0
        self._blink_phase = 0
        self._blink_phase_start = 0.0
        self._blink_count = 0
        self._blink_duration = 0.5

        self.centering = CenteringController(frame_w, frame_h)

    def start(self):
        self.state = InsertState.INIT
        print("[Insert] Миссия запущена (оптимизированная версия).")

    # ---- Вспомогательные методы ----
    def _load_targets(self):
        self.targets = self.marker_handler.map_data[:]
        print(f"[Insert] Загружено {len(self.targets)} целей.")
        for t in self.targets:
            is_cent = t.get('is_centered', False)
            print(f"  Цель ID 4x4={t['id_4x4']}: центрирована={is_cent}")
        return len(self.targets) > 0

    def _save_targets(self):
        self.marker_handler._save_map()

    def _remove_current_target(self):
        if self.current_target_idx < len(self.targets):
            removed = self.targets.pop(self.current_target_idx)
            for i, entry in enumerate(self.marker_handler.map_data):
                if entry['id_4x4'] == removed['id_4x4']:
                    del self.marker_handler.map_data[i]
                    break
            self._save_targets()
            print(f"[Insert] Цель {removed} удалена (маркер не найден).")
            return True
        return False

    def _update_current_target_position(self, is_centered=True):
        if self.current_target_idx < len(self.targets):
            pos = self.drone.get_position()
            if pos is not None:
                x, y, z = pos
                self.current_x, self.current_y, self.current_z = x, y, z
            else:
                x, y = self.current_x, self.current_y
            self.targets[self.current_target_idx]['pos'] = [x, y]
            self.targets[self.current_target_idx]['is_centered'] = is_centered
            for entry in self.marker_handler.map_data:
                if entry['id_4x4'] == self.targets[self.current_target_idx]['id_4x4']:
                    entry['pos'] = [x, y]
                    entry['is_centered'] = is_centered
                    break
            self._save_targets()
            print(f"[Insert] Обновлены координаты цели: ({x:.2f}, {y:.2f}) is_centered={is_centered}")

    def _set_all_leds(self, r, g, b):
        try:
            self.drone.drone.led_control(r=r, g=g, b=b)
        except AttributeError:
            pass

    def _blink_green(self):
        """Обновляет состояние мигания зелёным (вызывается каждый кадр)."""
        if not self._blink_active:
            return
        now = time.time()
        if now - self._blink_phase_start >= self._blink_duration:
            self._blink_phase = 1 - self._blink_phase
            self._blink_phase_start = now
            if self._blink_phase == 0:
                self._blink_count += 1
                if self._blink_count >= 3:
                    self._set_all_leds(0, 0, 0)
                    self._blink_active = False
                    return
            if self._blink_phase == 1:
                self._set_all_leds(0, 255, 0)
            else:
                self._set_all_leds(0, 0, 0)

    def _send_point(self, x, y, z=None, yaw=0.0):
        if z is None:
            z = self.current_z
        self.drone.set_manual_speed(0, 0, 0, 0)
        self.drone.go_to_point(x, y, z, yaw)
        self._move_sent = True
        self._target_reached = False
        self._target_reached_time = 0.0
        self._command_sent_time = time.time()
        self.current_x = x
        self.current_y = y
        self.current_z = z
        print(f"[Insert] Летим в ({x:.2f}, {y:.2f}, {z:.2f})")

    def _has_reached_point(self):
        if self._target_reached:
            return True
        if self.drone.point_reached():
            self._target_reached = True
            self._target_reached_time = time.time()
            print("[Insert] Точка достигнута (зафиксировано).")
            return True
        return False

    def _is_stopped(self, threshold=0.05, duration=0.5):
        if self._target_reached and time.time() - self._target_reached_time > 2.0:
            return True
        return self.drone.is_position_stable(threshold, duration)

    def _find_target_marker(self, markers, target_id_4x4):
        for m in markers:
            if m['type'] == '4x4' and int(m['id']) == int(target_id_4x4):
                return m
        return None

    def _decrease_height(self):
        new_z = max(CENTERING_DESCENT_ALTITUDE, self.current_z - HEIGHT_STEP)
        if new_z < self.current_z:
            print(f"[Insert] Плавное снижение: {self.current_z:.2f} -> {new_z:.2f} м")
            self.current_z = new_z
            self._send_point(self.current_x, self.current_y, z=self.current_z)

    def _is_target_already_centered(self):
        if self.current_target_idx < len(self.targets):
            return self.targets[self.current_target_idx].get('is_centered', False)
        return False

    # ---- Основной цикл ----
    def update(self, markers):
        self._blink_green()
        self.drone.update_position_history()

        pos = self.drone.get_position()
        if pos is not None:
            self.current_x, self.current_y, self.current_z = pos

        if self.state == InsertState.DONE:
            return "done"

        if self.state == InsertState.INIT:
            if self._load_targets():
                self.current_target_idx = 0
                self.state = InsertState.FLY_TO_GLOBAL
            else:
                self.state = InsertState.DONE
            return None

        elif self.state == InsertState.FLY_TO_GLOBAL:
            if self.current_target_idx >= len(self.targets):
                self.state = InsertState.RETURN_HOME
                return None

            target = self.targets[self.current_target_idx]

            if not self._move_sent:
                if self._is_target_already_centered():
                    print(f"[Insert] Цель {self.current_target_idx+1} уже центрирована. Лечу на высоту 1.5 м.")
                    self._send_point(target['pos'][0], target['pos'][1], z=CENTERING_DESCENT_ALTITUDE)
                else:
                    self._send_point(target['pos'][0], target['pos'][1], z=DEFAULT_ALTITUDE)
                return None

            if time.time() - self._command_sent_time > 30.0:
                print("[Insert] Таймаут FLY_TO_GLOBAL. Перехожу к стабилизации.")
                self._move_sent = False
                self.state = InsertState.STABILIZE
                self.centering_start_time = time.time()
                return None

            if self._has_reached_point():
                self._move_sent = False
                self.state = InsertState.STABILIZE
                self.centering_start_time = time.time()
                print("[Insert] Прибыли, стабилизация 2 сек.")
            return None

        elif self.state == InsertState.STABILIZE:
            if time.time() - self.centering_start_time < STABILIZE_TIME:
                return None
            print("[Insert] Стабилизация завершена.")
            if self._is_target_already_centered():
                self.state = InsertState.CENTERED_PAUSE
                self.centering_start_time = time.time()
                self._msg_printed = False
                # Мигание будет активировано в CENTERED_PAUSE
                return None
            else:
                self.state = InsertState.SEARCHING
                self.search_start_time = time.time()
            return None

        elif self.state == InsertState.SEARCHING:
            if time.time() - self.search_start_time > TIMEOUT_SEARCH:
                print(f"[Insert] ⏱ Время поиска истекло (>{TIMEOUT_SEARCH} сек). Удаляем цель.")
                if self._remove_current_target():
                    if len(self.targets) == 0:
                        print("[Insert] Все цели удалены. Возврат на базу.")
                        self.state = InsertState.RETURN_HOME
                        return None
                    else:
                        self.state = InsertState.FLY_TO_GLOBAL
                        self._move_sent = False
                        return None
                else:
                    self.state = InsertState.NEXT
                    return None

            target = self.targets[self.current_target_idx]
            marker = self._find_target_marker(markers, target['id_4x4'])
            if marker:
                print("[Insert] ✅ Маркер найден! Пауза перед центрированием.")
                self.drone.set_manual_speed(0, 0, 0, 0)
                self.state = InsertState.PRE_CENTERING_PAUSE
                self.centering_start_time = time.time()
                return None

            phase = (time.time() - self.search_start_time) % 10.0
            if phase < 4.0:
                self.drone.set_manual_speed(0, 0, 0, 0.25)
            elif phase < 8.0:
                self.drone.set_manual_speed(0, 0, 0, -0.25)
            else:
                self.drone.set_manual_speed(0, 0, 0, 0)
            return None

        elif self.state == InsertState.PRE_CENTERING_PAUSE:
            if time.time() - self.centering_start_time < CENTERING_PAUSE_BEFORE:
                return None
            print("[Insert] Пауза завершена. Начинаю центрирование.")
            target = self.targets[self.current_target_idx]
            marker = self._find_target_marker(markers, target['id_4x4'])
            if marker is None:
                print("[Insert] Маркер потерян. Перехожу в режим ожидания.")
                self.lost_during_movement = False
                self.state = InsertState.WAITING_FOR_MARKER
                self.waiting_start_time = time.time()
                return None

            self.centering_start_x = self.current_x
            self.centering_start_y = self.current_y
            self.centering.reset(self.centering_start_x, self.centering_start_y)
            self.centering.last_marker_pos = self.centering.get_marker_center(marker)
            self.state = InsertState.CENTERING_NEXT_MOVE
            return None

        elif self.state == InsertState.CENTERING_NEXT_MOVE:
            target = self.targets[self.current_target_idx]
            marker = self._find_target_marker(markers, target['id_4x4'])
            if marker is None:
                print("[Insert] Маркер потерян перед движением. Перехожу в ожидание.")
                self.lost_during_movement = False
                self.state = InsertState.WAITING_FOR_MARKER
                self.waiting_start_time = time.time()
                return None

            centered, stable = self.centering.check_centered_stable(marker)
            if centered:
                print(f"[Insert] ✅ Маркер отцентрирован! stable_frames={stable}")
                self.drone.set_manual_speed(0, 0, 0, 0)
                self.state = InsertState.POST_CENTERING_PAUSE
                self.centering_start_time = time.time()
                self._move_sent = False
                # Мигание будет активировано в POST_CENTERING_PAUSE
                self._send_point(self.current_x, self.current_y, z=CENTERING_DESCENT_ALTITUDE)
                return None

            step = self.centering.get_centering_step(marker)
            err_x, err_y = self.centering.get_current_error(marker)

            if self.centering.current_move_direction == (0.0, 0.0) or self.centering.attempts >= CENTERING_MAX_ATTEMPTS:
                if self.centering.attempts >= CENTERING_MAX_ATTEMPTS:
                    print("[Insert] Достигнут лимит попыток. Сбрасываю историю.")
                    self.centering.bad_directions = []
                    self.centering.attempts = 0
                dx, dy = self.centering.choose_direction(step, err_x, err_y)
                self.centering.current_move_direction = (dx, dy)
                self.centering.attempts = 0
                print(f"[Insert] Пробую направление: ({dx:.2f}, {dy:.2f}) на высоте {self.current_z:.2f}")
            else:
                dx, dy = self.centering.current_move_direction

            self.centering.last_marker_pos = self.centering.get_marker_center(marker)
            new_x = self.centering_start_x + dx
            new_y = self.centering_start_y + dy
            self._send_point(new_x, new_y)
            self.state = InsertState.CENTERING_WAITING
            return None

        elif self.state == InsertState.CENTERING_WAITING:
            if not self._has_reached_point():
                if time.time() - self._command_sent_time > 10.0:
                    print("[Insert] Таймаут движения в CENTERING_WAITING.")
                    self.state = InsertState.CENTERING_ANALYZE
                return None

            if not self._is_stopped():
                return None

            print("[Insert] Движение завершено и остановка стабильна. Анализируем.")
            self.state = InsertState.CENTERING_ANALYZE
            return None

        elif self.state == InsertState.CENTERING_ANALYZE:
            target = self.targets[self.current_target_idx]
            marker = self._find_target_marker(markers, target['id_4x4'])
            if marker is None:
                print("[Insert] Маркер потерян после движения. Откат к начальной точке центрирования.")
                self._send_point(self.centering_start_x, self.centering_start_y)
                if self.centering.current_move_direction != (0.0, 0.0):
                    self.centering.bad_directions.append(self.centering.current_move_direction)
                self.centering.current_move_direction = (0.0, 0.0)
                self.state = InsertState.ROLLBACK
                return None

            result, new_anchor_x, new_anchor_y = self.centering.analyze_after_move(
                marker, self.current_x, self.current_y, self.centering.current_move_direction
            )

            if result == 'centered':
                print("[Insert] ✅ Маркер уже центрирован! Завершаем.")
                self.drone.set_manual_speed(0, 0, 0, 0)
                self.state = InsertState.POST_CENTERING_PAUSE
                self.centering_start_time = time.time()
                self._move_sent = False
                self._send_point(self.current_x, self.current_y, z=CENTERING_DESCENT_ALTITUDE)
                return None

            elif result == 'good':
                self.centering_start_x = new_anchor_x
                self.centering_start_y = new_anchor_y
                self._decrease_height()
                self.state = InsertState.CENTERING_NEXT_MOVE
                return None

            elif result == 'bad':
                self._send_point(self.centering_start_x, self.centering_start_y)
                self.centering.current_move_direction = (0.0, 0.0)
                self.state = InsertState.ROLLBACK
                return None

            elif result == 'lost':
                self.lost_during_movement = True
                self.state = InsertState.WAITING_FOR_MARKER
                self.waiting_start_time = time.time()
                return None

        elif self.state == InsertState.WAITING_FOR_MARKER:
            target = self.targets[self.current_target_idx]
            marker = self._find_target_marker(markers, target['id_4x4'])
            if marker is not None:
                print("[Insert] ✅ Маркер снова виден! Продолжаю центрирование.")
                self.centering.last_marker_pos = self.centering.get_marker_center(marker)
                self.centering.attempts = 0
                self.centering.stable_frames = 0
                self.centering.current_move_direction = (0.0, 0.0)
                self.lost_during_movement = False
                self.state = InsertState.CENTERING_NEXT_MOVE
                return None

            timeout = TIMEOUT_MARKER_LOST_MOVEMENT if self.lost_during_movement else TIMEOUT_MARKER_LOST
            if time.time() - self.waiting_start_time > timeout:
                if self.lost_during_movement:
                    print(f"[Insert] ⏱ Маркер не появился за {TIMEOUT_MARKER_LOST_MOVEMENT}с. Откат.")
                    self.lost_during_movement = False
                    self._send_point(self.centering_start_x, self.centering_start_y)
                    self.state = InsertState.ROLLBACK
                    return None
                else:
                    print(f"[Insert] ⏱ Маркер не появился за {TIMEOUT_MARKER_LOST}с. Перехожу в поиск.")
                    self.lost_during_movement = False
                    self.state = InsertState.SEARCHING
                    self.search_start_time = time.time()
                    return None

            self.drone.set_manual_speed(0, 0, 0, 0)
            return None

        elif self.state == InsertState.ROLLBACK:
            if not self._has_reached_point():
                if time.time() - self._command_sent_time > 10.0:
                    print("[Insert] Таймаут отката, принудительный переход.")
                    self._move_sent = False
                    self.state = InsertState.CENTERING_NEXT_MOVE
                    self.centering.current_move_direction = (0.0, 0.0)
                    self.centering.attempts += 1
                return None
            if not self._is_stopped():
                return None
            print("[Insert] Откат выполнен. Продолжаю центрирование.")
            self.state = InsertState.CENTERING_NEXT_MOVE
            self.centering.current_move_direction = (0.0, 0.0)
            self.centering.attempts += 1
            return None

        elif self.state == InsertState.POST_CENTERING_PAUSE:
            if not self._move_sent:
                # Активируем мигание зелёным для первой цели (после центрирования)
                if self.current_target_idx == 0 and not self._blink_active:
                    self._blink_active = True
                    self._blink_start_time = time.time()
                    self._blink_phase = 0
                    self._blink_phase_start = time.time()
                    self._blink_count = 0
                    self._set_all_leds(0, 255, 0)
                    self._blink_phase = 1
                self._send_point(self.current_x, self.current_y, z=CENTERING_DESCENT_ALTITUDE)
                return None

            if not self._has_reached_point():
                if time.time() - self._command_sent_time > self.descent_timeout:
                    print("[Insert] Таймаут снижения, считаем достигнутым.")
                    self._move_sent = False
                return None

            if not self._is_stopped():
                return None

            if time.time() - self.centering_start_time < CENTERING_PAUSE_AFTER:
                if not self._msg_printed:
                    print("[Insert] Ожидание 10 сек на высоте 1.5 м...")
                    self._msg_printed = True
                return None

            print("[Insert] Пауза завершена. Обновляем карту.")
            self._msg_printed = False
            self._update_current_target_position(is_centered=True)
            self.state = InsertState.UPDATE_MAP
            return None

        elif self.state == InsertState.CENTERED_PAUSE:
            if not self._move_sent:
                # Активируем мигание зелёным для первой цели (если уже центрирована)
                if self.current_target_idx == 0 and not self._blink_active:
                    self._blink_active = True
                    self._blink_start_time = time.time()
                    self._blink_phase = 0
                    self._blink_phase_start = time.time()
                    self._blink_count = 0
                    self._set_all_leds(0, 255, 0)
                    self._blink_phase = 1
                self._send_point(self.current_x, self.current_y, z=CENTERING_DESCENT_ALTITUDE)
                return None

            if not self._has_reached_point():
                if time.time() - self._command_sent_time > self.descent_timeout:
                    print("[Insert] Таймаут снижения (уже центрирована).")
                    self._move_sent = False
                return None

            if not self._is_stopped():
                return None

            if time.time() - self.centering_start_time < CENTERING_PAUSE_AFTER:
                if not self._msg_printed:
                    print("[Insert] Цель уже центрирована. Ожидание 10 сек...")
                    self._msg_printed = True
                return None

            print("[Insert] Пауза завершена. Обновляем карту (is_centered=True).")
            self._msg_printed = False
            if self._blink_active:
                self._set_all_leds(0, 0, 0)
                self._blink_active = False
            self._update_current_target_position(is_centered=True)
            self.state = InsertState.UPDATE_MAP
            return None

        elif self.state == InsertState.UPDATE_MAP:
            target = self.targets[self.current_target_idx]
            self.marker_handler.check_and_record_markers(
                markers, (self.current_x, self.current_y),
                target_id_4x4=target['id_4x4'],
                is_centered=target.get('is_centered', False)
            )
            self.state = InsertState.NEXT
            return None

        elif self.state == InsertState.NEXT:
            self.current_target_idx += 1
            if self.current_target_idx < len(self.targets):
                self.state = InsertState.FLY_TO_GLOBAL
                self._move_sent = False
                self._target_reached = False
            else:
                print("[Insert] Все цели обработаны. Возврат на базу.")
                self._send_point(0, 0, z=DEFAULT_ALTITUDE)
                self.state = InsertState.RETURN_HOME
            return None

        elif self.state == InsertState.RETURN_HOME:
            if self._has_reached_point() and self._is_stopped():
                print("[Insert] Возврат на базу выполнен. Миссия завершена.")
                self.state = InsertState.DONE
            return None

        elif self.state == InsertState.DONE:
            return "done"

        return None

    def get_status(self):
        return "done" if self.state == InsertState.DONE else "running"
