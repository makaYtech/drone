import csv
import time

class FlightLogger:
    def __init__(self, filename="flight_log.csv"):
        self.filename = filename
        self.file = open(self.filename, mode='w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            'time_sec', 'event_type',
            'cmd_x', 'cmd_y', 'cmd_z', 'cmd_yaw',
            'cmd_vx', 'cmd_vy', 'cmd_vz', 'cmd_yaw_rate',
            'tel_x', 'tel_y', 'tel_z', 'tel_yaw',
            'comment'
        ])
        self.start_time = time.time()
        print(f"[Logger] Лог полета записывается в {self.filename}")

    def _get_rel_time(self):
        return round(time.time() - self.start_time, 3)

    def log_goto(self, x, y, z, yaw):
        self.writer.writerow([self._get_rel_time(), 'GOTO_POINT', x, y, z, yaw, '', '', '', '', '', '', '', '', ''])
        self.file.flush()

    def log_speed(self, vx, vy, vz, yaw_rate):
        self.writer.writerow([self._get_rel_time(), 'SET_SPEED', '', '', '', '', vx, vy, vz, yaw_rate, '', '', '', '', ''])
        self.file.flush()

    def log_telemetry(self, x, y, z, yaw, comment=""):
        self.writer.writerow([self._get_rel_time(), 'TELEMETRY', '', '', '', '', '', '', '', '', x, y, z, yaw, comment])
        self.file.flush()

    def log_event(self, event_name, comment=""):
        self.writer.writerow([self._get_rel_time(), 'EVENT', '', '', '', '', '', '', '', '', '', '', '', '', f"{event_name} | {comment}"])
        self.file.flush()

    def close(self):
        if self.file:
            self.file.close()
            print(f"[Logger] Лог сохранен.")
