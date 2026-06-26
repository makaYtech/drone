import sys
import os
import cv2
import time
import logging
import json
import numpy as np

from config import IP, PORT, PORT_CAM
from camera.camera_utils import CameraManager
from camera.aruco.detector import ArUcoDetector
from control.drone_global_control import DroneGlobalController

# ===== Декодер =====
INDEX_TO_PLANT = {
    0: "Томаты",
    1: "Огурцы",
    2: "Базилик"
}

def decode(code):
    """
    Декодирует число (0..248) в тип растения и количество.
    Возвращает (название_растения, количество) или (None, сообщение_об_ошибке).
    """
    if not (0 <= code <= 248):
        return None, "Код вне допустимого диапазона"
    idx = code // 83
    offset = code % 83
    if idx not in INDEX_TO_PLANT:
        return None, "Неверный индекс типа"
    quantity = offset + 8
    return INDEX_TO_PLANT[idx], quantity

def generate_report_from_map(map_file="map.json", output_file="report.txt"):
    """Читает map.json, декодирует пары и записывает отчёт."""
    if not os.path.exists(map_file):
        print(f"[Report] Файл {map_file} не найден.")
        return

    try:
        with open(map_file, 'r') as f:
            data = json.load(f)
    except Exception as e:
        print(f"[Report] Ошибка чтения {map_file}: {e}")
        return

    if not data:
        print("[Report] Нет данных для отчёта.")
        return

    lines = []
    lines.append("=== Отчёт по обнаруженным парам маркеров ===")
    lines.append("Тип растения | Количество | Координаты (x, y)")

    for entry in data:
        # Пропускаем одиночные маркеры, если они есть
        if entry.get('type') == 'single':
            continue

        id_4x4 = entry.get('id_4x4')
        id_5x5 = entry.get('id_5x5')
        pos = entry.get('pos', [0, 0])
        x, y = pos[0], pos[1]

        # Декодируем ID второго маркера (5x5)
        plant, quantity = decode(id_5x5)
        if plant is None:
            lines.append(f"ОШИБКА: ID 5x5={id_5x5} не декодируется: {quantity}")
            continue

        # Выводим без проверки соответствия, просто по декодированным данным
        lines.append(f"{plant:12} | {quantity:10} | ({x:.2f}, {y:.2f})")

    lines.append("=== Конец отчёта ===")

    with open(output_file, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines))

    print(f"[Report] Отчёт сохранён в {output_file}")

# ===== Фильтр вывода (оставляем как есть) =====
class FilterOutManualSpeed:
    def __init__(self, stream):
        self.stream = stream
        self.last_was_manual = False
    def write(self, text):
        if 'MANUAL_SPEED' in text:
            self.last_was_manual = True
            return
        if self.last_was_manual and text.strip() == '':
            self.last_was_manual = False
            return
        self.last_was_manual = False
        self.stream.write(text)
    def flush(self):
        self.stream.flush()

sys.stdout = FilterOutManualSpeed(sys.stdout)
logging.getLogger('pioneer_sdk').setLevel(logging.WARNING)

def main():
    TARGET_GROUPS = 4
    MAP_FILE = "map.json"
    
    # 1. Инициализация
    print("[Main] Инициализация компонентов...")
    cam_mgr = CameraManager(IP, PORT_CAM)
    detector = ArUcoDetector()
    drone = DroneGlobalController(IP, PORT, simulator=True)
    
# 2. ВЫБОР РЕЖИМА
    if os.path.exists(MAP_FILE) and os.path.getsize(MAP_FILE) > 2:
        print(f"[Main] Обнаружен существующий {MAP_FILE}. Запуск режима INSERT.")
        from outside.insert import InsertMission
        mission = InsertMission(drone, map_file=MAP_FILE, frame_w=640, frame_h=480)
    else:
        print(f"[Main] {MAP_FILE} не найден или пуст. Запуск режима EXPLORATION (поиск).")
        from outside import Mission
        mission = Mission(drone, initial_yaw=0.0, target_groups=TARGET_GROUPS, default_altitude=1.7)   # <-- здесь высота 1.7

    # ... (дальше)
    # 3. Предполетная подготовка
    drone.arm()
    drone.takeoff()
    print("[Main] Дрон взлетел, запускаем миссию...")
    mission.start()
    
    if type(mission).__name__ == 'Mission':
        drone.go_to_point(0, 0, 1.7, np.radians(180))

    # 4. Главный цикл
    try:
        while True:
            frame = cam_mgr.get_frame()
            if frame is None:
                time.sleep(0.02)
                continue

            vis_frame, markers = detector.process_frame(frame)
            status = mission.update(markers)

            if status == 'done':
                print("\n[Main] Миссия завершена, выполняем посадку...")
                if hasattr(mission, 'get_results'):
                    results = mission.get_results()
                    print(f"[Main] Собрано групп: {len(results)}")
                
                # Генерируем отчёт по карте
                generate_report_from_map(MAP_FILE, "report.txt")
                
                drone.land()
                time.sleep(2)
                break

            cv2.imshow("ArUco Detection", vis_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\n[Main] Принудительная остановка по нажатию 'q'")
                drone.land()
                break

    except KeyboardInterrupt:
        print("\n[Main] Остановка по Ctrl+C")
        drone.land()
    except Exception as e:
        print(f"\n[Main] Критическая ошибка: {e}")
        drone.land()
    finally:
        cv2.destroyAllWindows()
        cam_mgr.release()
        print("[Main] Программа завершена.")

if __name__ == "__main__":
    main()