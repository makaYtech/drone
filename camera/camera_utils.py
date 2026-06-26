from pioneer_sdk import Camera

class CameraManager:
    def __init__(self, ip, port, log_connection=True):
        self.camera = Camera(ip=ip, port=port, log_connection=log_connection)

    def get_frame(self):
        return self.camera.get_cv_frame()

    def release(self):
        pass
