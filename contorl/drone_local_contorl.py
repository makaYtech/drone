class DroneLocalController:
    """Будущая реализация для точного позиционирования относительно маркера."""
    def __init__(self, global_controller):
        self.global_ctrl = global_controller

    def hover_at_marker(self, coords):
        # Здесь будет локальное управление (например, PID)
        pass
