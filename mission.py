from enum import Enum
import time
import numpy as np
from collections import defaultdict

class MissionState(Enum):
    IDLE = 0
    TAKEOFF = 1
    FLYING_TO_FIRST = 2
    COLLECTING = 3
    FLYING_TO_SECOND = 4
    COMPLETE = 5

class Mission:
    def __init__(self, drone_controller):
        self.drone = drone_controller
        self.state = MissionState.IDLE
        self.collected_markers = []
        self.buffer = []
        self.collect_start_time = None
        self.COLLECT_DURATION = 3.0
        self.FORWARD_SPEED = 0.4
        self.MIN_CONFIRMATIONS = 3

    def start(self):
        if self.state == MissionState.IDLE:
            self.state = MissionState.TAKEOFF
            print("Миссия: взлёт...")

    def update(self, markers_list):
        if self.state == MissionState.IDLE:
            return 'idle'

        elif self.state == MissionState.TAKEOFF:
            self.state = MissionState.FLYING_TO_FIRST
            print("Миссия: поиск первой группы маркеров...")
            self.drone.set_manual_speed(vx=0, vy=self.FORWARD_SPEED, vz=0, yaw_rate=0)
            return None

        elif self.state == MissionState.FLYING_TO_FIRST:
            if len(markers_list) > 0:
                self.state = MissionState.COLLECTING
                self.buffer = []
                self.collect_start_time = time.time()
                print("Миссия: остановка для сбора данных...")
                self.drone.hold_position()
            else:
                self.drone.set_manual_speed(vx=0, vy=self.FORWARD_SPEED, vz=0, yaw_rate=0)
            return None

        elif self.state == MissionState.COLLECTING:
            self.buffer.append(markers_list)
            if time.time() - self.collect_start_time > self.COLLECT_DURATION:
                final_markers = self._process_buffer()
                if final_markers:
                    existing_ids = {m['id'] for m in self.collected_markers}
                    for m in final_markers:
                        if m['id'] not in existing_ids:
                            self.collected_markers.append(m)
                            existing_ids.add(m['id'])
                    print(f"Миссия: собрано {len(self.collected_markers)} маркеров.")

                if len(self.collected_markers) >= 2:
                    self.state = MissionState.COMPLETE
                    print("Миссия: собрано 2 маркера, завершаем.")
                    self.drone.hold_position()
                    return 'done'
                else:
                    self.state = MissionState.FLYING_TO_SECOND
                    print("Миссия: продолжаем поиск...")
                    self.drone.set_manual_speed(vx=0, vy=self.FORWARD_SPEED, vz=0, yaw_rate=0)
            return None

        elif self.state == MissionState.FLYING_TO_SECOND:
            if len(markers_list) > 0:
                self.state = MissionState.COLLECTING
                self.buffer = []
                self.collect_start_time = time.time()
                print("Миссия: остановка для сбора данных...")
                self.drone.hold_position()
            else:
                self.drone.set_manual_speed(vx=0, vy=self.FORWARD_SPEED, vz=0, yaw_rate=0)
            return None

        elif self.state == MissionState.COMPLETE:
            print("Миссия завершена. Собранные маркеры:")
            for i, m in enumerate(self.collected_markers, 1):
                print(f"  {i}: ID={m['id']}, coords={m['coords']}")
            return 'done'

        return None

    def _process_buffer(self):
        if not self.buffer:
            return []

        id_counter = defaultdict(int)
        id_coords = defaultdict(list)

        for frame_markers in self.buffer:
            for m in frame_markers:
                if m['coords'] is not None:
                    id_counter[m['id']] += 1
                    id_coords[m['id']].append(m['coords'])

        result = []
        for marker_id, count in id_counter.items():
            if count >= self.MIN_CONFIRMATIONS:
                coords_array = np.array(id_coords[marker_id])
                avg_coords = np.mean(coords_array, axis=0).tolist()
                result.append({
                    'id': marker_id,
                    'coords': avg_coords
                })
        return result

    def get_results(self):
        return self.collected_markers
