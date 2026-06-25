import sys
import cv2
import time
import logging
import numpy as np

from config import IP, PORT, PORT_CAM
from camera.camera_utils import CameraManager
from camera.aruco.detector import ArUcoDetector
from control.drone_global_control import DroneGlobalController
from outside import Mission

# ---- Фильтр вывода для скрытия спама MANUAL_SPEED от SDK ----
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
# -------------------------------------------------------------

# Уменьшаем шум от pioneer_sdk в логах
logging.getLogger('pioneer_sdk').setLevel(logging.WARNING)

def main():
    TARGET_GROUPS = 4
    
    # 1. Инициализация компонентов
    print("[Main] Инициализация компонентов...")
    cam_mgr = CameraManager(IP, PORT_CAM)
    detector = ArUcoDetector()
    drone = DroneGlobalController(IP, PORT, simulator=True)
    
    # 2. Создание миссии
    # Передаем контроллер дрона, миссия будет использовать его через Navigator
    mission = Mission(drone, initial_yaw=0.0, target_groups=TARGET_GROUPS)

    # 3. Предполетная подготовка
    drone.arm()
    drone.takeoff()
    print("[Main] Дрон взлетел, запускаем миссию...")
    
    # Запуск стейт-машины (переводит в состояние WAITING)
    mission.start()
    
    # Поворачиваем дрон на 180 градусов (используем обертку из DroneGlobalController)
    drone.go_to_point(0, 0, 2, np.radians(180))

    # 4. Главный цикл
    try:
        while True:
            frame = cam_mgr.get_frame()
            if frame is None:
                time.sleep(0.02)
                continue

            # Обработка кадра и поиск маркеров
            vis_frame, markers = detector.process_frame(frame)
            
            # Обновление стейт-машины миссии
            status = mission.update(markers)

            # Проверка завершения миссии
            if status == 'done':
                print("\n[Main] Миссия завершена, выполняем посадку...")
                results = mission.get_results()
                print(f"[Main] Собрано групп: {len(results)}")
                for i, r in enumerate(results, 1):
                    print(f"  {i}: 4x4 ID={r['id_4x4']}, 5x5 ID={r['id_5x5']}, pos={r['pos']}")
                
                drone.land()
                time.sleep(2)
                break

            # Отображение кадра с отрисованными маркерами
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
