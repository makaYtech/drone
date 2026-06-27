class PathPlanner:
    def __init__(self, square_size: float = 10.0, spiral_step: float = 1.0):
        self.SQUARE_SIZE = square_size
        self.SPIRAL_STEP = spiral_step
        self.spiral_points = []

    def generate_spiral_points(self, square_offset: tuple):
        """Генерирует НЕПРЕРЫВНУЮ спираль от центра к краям."""
        x_off, y_off = square_offset
        cx = x_off + self.SQUARE_SIZE / 2
        cy = y_off + self.SQUARE_SIZE / 2
        step = self.SPIRAL_STEP
        num_rings = int((self.SQUARE_SIZE / 2.0) / step)

        points = [(round(cx, 2), round(cy, 2))]

        for ring in range(1, num_rings + 1):
            x_min = round(max(x_off, cx - ring * step), 2)
            x_max = round(min(x_off + self.SQUARE_SIZE, cx + ring * step), 2)
            y_min = round(max(y_off, cy - ring * step), 2)
            y_max = round(min(y_off + self.SQUARE_SIZE, cy + ring * step), 2)

            last_x, last_y = points[-1]

            while last_y < y_max - 0.01:
                last_y = round(min(last_y + step, y_max), 2)
                points.append((last_x, last_y))

            while last_x > x_min + 0.01:
                last_x = round(max(last_x - step, x_min), 2)
                points.append((last_x, last_y))

            while last_x < x_max - 0.01:
                last_x = round(min(last_x + step, x_max), 2)
                points.append((last_x, last_y))

            while last_y > y_min + 0.01:
                last_y = round(max(last_y - step, y_min), 2)
                points.append((last_x, last_y))

            while last_x > x_min + 0.01:
                last_x = round(max(last_x - step, x_min), 2)
                points.append((last_x, last_y))

            target_y = round(y_max - step, 2) if ring < num_rings else y_max
            while last_y < target_y - 0.01:
                last_y = round(min(last_y + step, target_y), 2)
                points.append((last_x, last_y))

        unique = []
        seen = set()
        for p in points:
            r = (round(p[0], 2), round(p[1], 2))
            if r not in seen:
                seen.add(r)
                unique.append(r)

        corners = [
            (round(x_off, 2), round(y_off, 2)),
            (round(x_off, 2), round(y_off + self.SQUARE_SIZE, 2)),
            (round(x_off + self.SQUARE_SIZE, 2), round(y_off, 2)),
            (round(x_off + self.SQUARE_SIZE, 2), round(y_off + self.SQUARE_SIZE, 2))
        ]
        for c in corners:
            if c not in seen:
                unique.append(c)

        self.spiral_points = unique
        print(f"[PathPlanner] Квадрат {square_offset}: {len(self.spiral_points)} точек")

    def find_straight_line_end(self, start_idx: int) -> int:
        """Если 3+ точек идут строго по одной оси — возвращает индекс последней."""
        if start_idx >= len(self.spiral_points) - 2:
            return start_idx

        p0 = self.spiral_points[start_idx]
        p1 = self.spiral_points[start_idx + 1]

        dx = round(p1[0] - p0[0], 2)
        dy = round(p1[1] - p0[1], 2)

        if abs(dx) > 0.01 and abs(dy) < 0.01:
            axis = 'x'
            sign = 1 if dx > 0 else -1
        elif abs(dy) > 0.01 and abs(dx) < 0.01:
            axis = 'y'
            sign = 1 if dy > 0 else -1
        else:
            return start_idx

        end_idx = start_idx + 1
        for i in range(start_idx + 2, len(self.spiral_points)):
            curr = self.spiral_points[i]
            prev = self.spiral_points[i - 1]

            d_x = round(curr[0] - prev[0], 2)
            d_y = round(curr[1] - prev[1], 2)

            if axis == 'x':
                if abs(d_y) > 0.01 or (d_x * sign) < 0.01:
                    break
            else:
                if abs(d_x) > 0.01 or (d_y * sign) < 0.01:
                    break

            end_idx = i

        return end_idx if (end_idx - start_idx + 1) >= 3 else start_idx