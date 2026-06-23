import json
import os

class MapManager:
    def __init__(self, filename="map.json"):
        self.filename = filename
        self.data = []   # список групп: [{'id_4x4': int, 'id_6x6': int, 'pos': [x,y,z]}]

    def load(self):
        if os.path.exists(self.filename):
            with open(self.filename, 'r') as f:
                self.data = json.load(f)
            return self.data
        return []

    def save(self, data):
        with open(self.filename, 'w') as f:
            json.dump(data, f, indent=4)

    def add_group(self, id4, id6, pos):
        self.data.append({
            'id_4x4': id4,
            'id_6x6': id6,
            'pos': pos  # [x, y, z] координаты дрона над парой
        })
        self.save(self.data)
