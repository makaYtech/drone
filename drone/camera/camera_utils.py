from pioneer_sdk import Camera

class CameraManager:
    def __init__(self, ip, port, log_connection=True):
        self.camera = Camera(ip=ip, port=port, log_connection=log_connection)

    def get_frame(self):
        """Возвращает кадр (numpy array) или None."""
        return self.camera.get_cv_frame()

    def release(self):
        """Освобождение ресурсов (если потребуется)."""
        pass
