import cv2
import time
from config import IP, PORT, PORT_CAM
from camera import CameraManager, ArUcoDetector
from control import DroneGlobalController

def main():
    cam_mgr = CameraManager(IP, PORT_CAM)
    detector = ArUcoDetector()
    drone = DroneGlobalController(IP, PORT, simulator=True)

    drone.arm()
    drone.takeoff()
    print("Дрон взлетел, начинаем поиск маркера...")

    send_manual = False

    while True:
        try:
            frame = cam_mgr.get_frame()
            if frame is None:
                time.sleep(0.02)
                continue

            vis_frame, coords, x_center, y_center, ids = detector.process_frame(frame)

            if coords is not None:
                cmd, need = drone.compute_control(coords, x_center, frame.shape[1])
                if need:
                    drone.set_manual_speed(*cmd)
                    send_manual = True
                elif send_manual:
                    drone.hold_position()
                    send_manual = False

            cv2.imshow("ArUco Detection", vis_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        except Exception as e:
            print("Ошибка в цикле:", e)
            time.sleep(0.1)

    drone.land()
    cv2.destroyAllWindows()
    print("Миссия завершена.")

if __name__ == "__main__":
    main()
