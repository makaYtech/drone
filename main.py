import sys
import os
import cv2
import time
import logging
import numpy as np

from config import IP, PORT, PORT_CAM
from constants import DEFAULT_ALTITUDE, WAIT_BEFORE_START
from camera.camera_utils import CameraManager
from camera.aruco.detector import ArUcoDetector
from control.drone_global_control import DroneGlobalController
from utils import generate_report_from_map

# Настройка логирования
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Фильтр для подавления сообщений MANUAL_SPEED в stdout
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

def select_mission(drone, map_file="map.json"):
    """Выбирает режим миссии в зависимости от наличия map.json."""
    if os.path.exists(map_file) and os.path.getsize(map_file) > 2:
        logger.info("Обнаружен существующий %s. Запуск режима INSERT.", map_file)
        from outside.insert import InsertMission
        from outside.marker_handler import MarkerHandler
        marker_handler = MarkerHandler(map_file, square_size=10.0)
        return InsertMission(drone, marker_handler, frame_w=640, frame_h=480)
    else:
        logger.info("%s не найден или пуст. Запуск режима EXPLORATION.", map_file)
        from outside import Mission
        return Mission(drone, initial_yaw=0.0, target_groups=4, default_altitude=DEFAULT_ALTITUDE)


def main():
    MAP_FILE = "map.json"

    # 1. Инициализация
    logger.info("Инициализация компонентов...")
    cam_mgr = CameraManager(IP, PORT_CAM)
    detector = ArUcoDetector()
    drone = DroneGlobalController(IP, PORT, simulator=True)

    # 2. Выбор миссии
    mission = select_mission(drone, MAP_FILE)

    # 3. Предполётная подготовка
    drone.arm()
    drone.takeoff()
    logger.info("Дрон взлетел, запускаем миссию...")
    mission.start()

    # Если миссия разведки – развернуть дрон на 180° (опционально)
    if type(mission).__name__ == 'Mission':
        drone.go_to_point(0, 0, DEFAULT_ALTITUDE, np.radians(180))

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
                logger.info("Миссия завершена, выполняем посадку...")
                if hasattr(mission, 'get_results'):
                    results = mission.get_results()
                    logger.info("Собрано групп: %d", len(results))

                generate_report_from_map(MAP_FILE, "report.txt")
                drone.land()
                time.sleep(2)
                break

            cv2.imshow("ArUco Detection", vis_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                logger.info("Принудительная остановка по нажатию 'q'")
                drone.land()
                break

    except KeyboardInterrupt:
        logger.info("Остановка по Ctrl+C")
        drone.land()
    except Exception as e:
        logger.error("Критическая ошибка: %s", e, exc_info=True)
        drone.land()
    finally:
        cv2.destroyAllWindows()
        cam_mgr.release()
        logger.info("Программа завершена.")


if __name__ == "__main__":
    main()
