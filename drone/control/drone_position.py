def center_on_marker(self, marker_coords, frame_width, frame_height, 
                     target_distance=1.0, kp_xy=0.5, kp_z=0.5, max_speed=0.5):
    """
    Центрирует дрон по метке.
    
    Параметры:
        marker_coords – список [x, y, z] в системе камеры (метры)
        frame_width, frame_height – размеры кадра (пиксели)
        target_distance – желаемое расстояние до метки (метры)
        kp_xy – коэффициент пропорциональности для движения влево-вправо и вверх-вниз
        kp_z – коэффициент для движения вперёд-назад
        max_speed – максимальная скорость (м/с)
    
    Возвращает:
        кортеж (vx, vy, vz, yaw_rate) – скорости, которые нужно отправить дрону
    """
    if marker_coords is None:
        return (0, 0, 0, 0)
    x, y, z = marker_coords
    vx = max(-max_speed, min(max_speed, vx))
    vy = kp_xy * y
    vz = -kp_xy * y
    vz = max(-max_speed, min(max_speed, vz))
    vy = -kp_z * (z - target_distance)
    vy = kp_z * (z - target_distance)
    vy = max(-max_speed, min(max_speed, vy))
    return (vx, vy, vz, 0)
