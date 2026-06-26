import json
import math
from typing import List, Dict, Tuple

class MarkerHandler:
    def __init__(self, map_file: str = "map.json", square_size: float = 10.0):
        self.map_file = map_file
        self.square_size = square_size
        self.map_data = []          # основное хранилище
        self.collected = self.map_data  # для обратной совместимости
        self.recorded_pairs = set()
        self.recorded_singles = set()
        self.last_recorded_pos = {}
        self._load_existing_map()

    def _load_existing_map(self):
        try:
            with open(self.map_file, 'r') as f:
                data = json.load(f)
                if isinstance(data, list):
                    self.map_data = data
                    self.collected = self.map_data
                    for entry in data:
                        if entry.get('type') == 'pair':
                            self.recorded_pairs.add((entry['id_4x4'], entry['id_5x5']))
                        elif entry.get('type') == 'single':
                            self.recorded_singles.add(entry['id_4x4'])
                        pos = entry.get('pos')
                        if pos and len(pos) >= 2:
                            self.last_recorded_pos[entry['id_4x4']] = (pos[0], pos[1])
        except FileNotFoundError:
            pass
        except Exception as e:
            print(f"[MarkerHandler] Ошибка загрузки {self.map_file}: {e}")

    def _save_map(self):
        with open(self.map_file, 'w') as f:
            json.dump(self.map_data, f, indent=2)

    def _find_entry_by_id(self, id_4x4):
        for i, entry in enumerate(self.map_data):
            if entry['id_4x4'] == id_4x4:
                return i
        return None

    def _update_or_add_entry(self, entry):
        idx = self._find_entry_by_id(entry['id_4x4'])
        new_dist = entry.get('distance_to_marker', float('inf'))

        if idx is not None:
            old_entry = self.map_data[idx]
            old_dist = old_entry.get('distance_to_marker', float('inf'))

            if new_dist >= old_dist:
                return False

            if old_entry.get('type') == 'pair':
                self.recorded_pairs.discard((old_entry['id_4x4'], old_entry['id_5x5']))
            elif old_entry.get('type') == 'single':
                self.recorded_singles.discard(old_entry['id_4x4'])

            self.map_data[idx] = entry

            if entry.get('type') == 'pair':
                self.recorded_pairs.add((entry['id_4x4'], entry['id_5x5']))
            elif entry.get('type') == 'single':
                self.recorded_singles.add(entry['id_4x4'])

            self.last_recorded_pos[entry['id_4x4']] = tuple(entry['pos'])
            return True
        else:
            self.map_data.append(entry)
            if entry.get('type') == 'pair':
                self.recorded_pairs.add((entry['id_4x4'], entry['id_5x5']))
            elif entry.get('type') == 'single':
                self.recorded_singles.add(entry['id_4x4'])
            self.last_recorded_pos[entry['id_4x4']] = tuple(entry['pos'])
            return True

    def check_and_record_markers(self, markers: List[Dict], marker_position: Tuple[float, float]):
        """
        Сохраняет маркеры с указанной позицией marker_position (вычисленной реальной координатой).
        """
        m4 = [m for m in markers if m['type'] == '4x4' and m['coords'] is not None]
        m5 = [m for m in markers if m['type'] == '5x5' and m['coords'] is not None]

        if m4 and m5:
            for marker4 in m4:
                for marker5 in m5:
                    if marker4['id'] == marker5['id']:
                        entry = {
                            'type': 'pair',
                            'id_4x4': marker4['id'],
                            'id_5x5': marker5['id'],
                            'marker_coords_4x4': marker4['coords'],
                            'marker_coords_5x5': marker5['coords'],
                            'pos': list(marker_position),
                            'distance_to_marker': marker4['coords'][2]
                        }
                        updated = self._update_or_add_entry(entry)
                        if updated:
                            self._save_map()
                            print(f"[MarkerHandler] Записана/обновлена пара: 4x4={marker4['id']}, 5x5={marker5['id']} на {marker_position}")
                        return

        # Одиночные 4x4
        for marker4 in m4:
            idx = self._find_entry_by_id(marker4['id'])
            if idx is not None and self.map_data[idx].get('type') == 'pair':
                continue

            entry = {
                'type': 'single',
                'id_4x4': marker4['id'],
                'id_5x5': None,
                'marker_coords_4x4': marker4['coords'],
                'marker_coords_5x5': None,
                'pos': list(marker_position),
                'distance_to_marker': marker4['coords'][2]
            }
            updated = self._update_or_add_entry(entry)
            if updated:
                self._save_map()
                print(f"[MarkerHandler] Записана/обновлена одиночная 4x4 ID={marker4['id']} на {marker_position}")
            return

    def is_in_square(self, pos: Tuple[float, float], square_offset: Tuple[float, float]) -> bool:
        x_off, y_off = square_offset
        x, y = pos[0], pos[1]
        return (x_off <= x <= x_off + self.square_size and
                y_off <= y <= y_off + self.square_size)

    def get_results(self) -> List[Dict]:
        return self.map_data