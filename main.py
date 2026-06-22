import cv2
import time
from config import IP, PORT, PORT_CAM
from camera import CameraManager, ArUcoDetector
from control import DroneGlobalController
from mission import Mission

def main():
    cam_mgr = CameraManager(IP, PORT_CAM)
    detector = ArUcoDetector()
    drone = DroneGlobalController(IP, PORT, simulator=True)
    mission = Mission(drone)

    drone.arm()
    drone.takeoff()
    print("Дрон взлетел, запускаем миссию...")
    mission.start()

    while True:
        try:
            frame = cam_mgr.get_frame()
            if frame is None:
                time.sleep(0.02)
                continue

            vis_frame, markers = detector.process_frame(frame)
            status = mission.update(markers)

            if status == 'done':
                print("Миссия завершена, посадка...")
                results = mission.get_results()
                print("Результаты:", results)
                drone.land()
                time.sleep(2)
                break

            cv2.imshow("ArUco Detection", vis_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        except Exception as e:
            print("Ошибка:", e)
            time.sleep(0.1)

    cv2.destroyAllWindows()
    drone.disarm()
    print("Программа завершена.")

if __name__ == "__main__":
    main()
