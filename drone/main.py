import sys
import cv2
import time
import logging
import numpy as np
from config import IP, PORT, PORT_CAM
from camera import CameraManager, ArUcoDetector
from control import DroneGlobalController
from mission import Mission

# ---- Фильтр вывода для скрытия MANUAL_SPEED ----
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
# -------------------------------------------------

logging.getLogger('pioneer_sdk').setLevel(logging.WARNING)

def main():
    # Режим: 'explore' – первый запуск (сбор карты), 'navigate' – последующие
    MODE = 'explore'
    TARGET_GROUPS = 4
    
    cam_mgr = CameraManager(IP, PORT_CAM)
    detector = ArUcoDetector()
    drone = DroneGlobalController(IP, PORT, simulator=True)
    mission = Mission(drone, target_groups=TARGET_GROUPS)

    drone.arm()
    drone.takeoff()
    print("Дрон взлетел, запускаем миссию...")
    mission.start()
    #drone.drone.go_to_local_point(0, 0, 2, np.radians(180))

    while True:
        try:
            frame = cam_mgr.get_frame()
            if frame is None:
                time.sleep(0.02)
                continue

            vis_frame, markers = detector.process_frame(frame)
            
            # Обновляем гашение инерции (если метод есть в DroneGlobalController)
            if hasattr(drone, 'update_inertia'):
                drone.update_inertia()
            
            status = mission.update(markers)

            if status == 'done':
                print("Миссия завершена, посадка...")
                results = mission.get_results()
                print("Собрано групп: ", len(results))
                for i, r in enumerate(results, 1):
                    print(f"  {i}: 4x4 ID={r['id_4x4']}, 5x5 ID={r['id_5x5']}, pos={r['drone_pos']}")
                drone.land()
                time.sleep(2)
                break

            cv2.imshow("ArUco Detection", vis_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                drone.land()
                break

        except Exception as e:
            print("Ошибка: ", e)
            time.sleep(0.1)

    cv2.destroyAllWindows()
    print("Программа завершена.")

if __name__ == "__main__":
    main()
