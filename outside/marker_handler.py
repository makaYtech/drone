import json
from typing import List, Dict, Tuple

class MarkerHandler:
    def __init__(self, map_file: str = "map.json", square_size: float = 10.0):
        self.map_file = map_file
        self.square_size = square_size
        self.collected = []
        self.map_data = []
        self.recorded_pairs = set()

    def check_and_record_markers(self, markers: List[Dict], current_target_pos: Tuple[float, float]):
        """Ищет пары 4x4 и 5x5, записывает их и сохраняет карту."""
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
                            'pos': list(current_target_pos)
                        }
                        self.collected.append(entry)
                        self.map_data.append(entry)
                        self.recorded_pairs.add(pair_id)

                        with open(self.map_file, "w") as f:
                            json.dump(self.map_data, f, indent=2)

                        print(f"[MarkerHandler] Записана пара: 4x4={marker4['id']}, "
                              f"5x5={marker5['id']} на {current_target_pos}")
                        return

    def is_in_square(self, pos: Tuple[float, float], square_offset: Tuple[float, float]) -> bool:
        """Проверяет, находится ли точка внутри заданного квадрата."""
        x_off, y_off = square_offset
        x, y = pos[0], pos[1]
        return (x_off <= x <= x_off + self.square_size and
                y_off <= y <= y_off + self.square_size)

    def get_results(self) -> List[Dict]:
        return self.collected
