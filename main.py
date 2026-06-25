import sys
import os
import cv2
import time
import logging
import numpy as np

from config import IP, PORT, PORT_CAM
from camera.camera_utils import CameraManager
from camera.aruco.detector import ArUcoDetector
from control.drone_global_control import DroneGlobalController

# Фильтр вывода (оставляем как есть)
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
    
    # 1. Инициализация железа
    print("[Main] Инициализация компонентов...")
    cam_mgr = CameraManager(IP, PORT_CAM)
    detector = ArUcoDetector()
    drone = DroneGlobalController(IP, PORT, simulator=True)
    
    # 2. ВЫБОР РЕЖИМА
    # Проверяем, есть ли уже карта (файл существует и его размер > 2 байт, т.е. не пустой список "[]")
    if os.path.exists(MAP_FILE) and os.path.getsize(MAP_FILE) > 2:
        print(f"[Main] Обнаружен существующий {MAP_FILE}. Запуск режима INSERT.")
        from outside.insert import InsertMission
        mission = InsertMission(drone, map_file=MAP_FILE, frame_w=640, frame_h=480)
    else:
        print(f"[Main] {MAP_FILE} не найден или пуст. Запуск режима EXPLORATION (поиск).")
        from outside import Mission
        mission = Mission(drone, initial_yaw=0.0, target_groups=TARGET_GROUPS)

    # 3. Предполетная подготовка
    drone.arm()
    drone.takeoff()
    print("[Main] Дрон взлетел, запускаем миссию...")
    mission.start()
    
    if type(mission).__name__ == 'Mission':
        # Поворот на 180 только для режима поиска
        drone.go_to_point(0, 0, 2, np.radians(180))

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
