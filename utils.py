import json
import os
from typing import Optional, Tuple, List, Dict

INDEX_TO_PLANT = {
    0: "Томаты",
    1: "Огурцы",
    2: "Базилик"
}

def decode(code: int) -> Tuple[Optional[str], Optional[str]]:
    """
    Декодирует число (0..248) в тип растения и количество.
    Возвращает (название_растения, количество) или (None, сообщение_об_ошибке).
    """
    if not (0 <= code <= 248):
        return None, "Код вне допустимого диапазона"
    idx = code // 83
    offset = code % 83
    if idx not in INDEX_TO_PLANT:
        return None, "Неверный индекс типа"
    quantity = offset + 8
    return INDEX_TO_PLANT[idx], str(quantity)


def generate_report_from_map(map_file: str = "map.json", output_file: str = "report.txt") -> None:
    """Читает map.json, декодирует пары и записывает отчёт."""
    if not os.path.exists(map_file):
        print(f"[Report] Файл {map_file} не найден.")
        return

    try:
        with open(map_file, 'r') as f:
            data = json.load(f)
    except Exception as e:
        print(f"[Report] Ошибка чтения {map_file}: {e}")
        return

    if not data:
        print("[Report] Нет данных для отчёта.")
        return

    lines = []
    lines.append("=== Отчёт по обнаруженным парам маркеров ===")
    lines.append("Тип растения | Количество | Координаты (x, y)")

    for entry in data:
        if entry.get('type') == 'single':
            continue

        id_4x4 = entry.get('id_4x4')
        id_5x5 = entry.get('id_5x5')
        pos = entry.get('pos', [0, 0])
        x, y = pos[0], pos[1]

        plant, quantity = decode(id_5x5)
        if plant is None:
            lines.append(f"ОШИБКА: ID 5x5={id_5x5} не декодируется: {quantity}")
            continue

        lines.append(f"{plant:12} | {quantity:10} | ({x:.2f}, {y:.2f})")

    lines.append("=== Конец отчёта ===")

    with open(output_file, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines))

    print(f"[Report] Отчёт сохранён в {output_file}")
