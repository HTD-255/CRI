"""
Driving Scenarios - Tạo quỹ đạo target
"""

import numpy as np
from scipy.interpolate import CubicSpline


def get_lateral_trajectory(name):
    """Tạo waypoints cho các kịch bản"""

    if name == 'lateral_zigzag':
        y = np.linspace(-20, 80, 30)
        x = np.zeros_like(y)
        for i in range(len(y)):
            amp = 15 * (1 - i / (len(y)-1) * 0.3)
            x[i] = amp * np.sin(0.3 * y[i])
        cs = CubicSpline(y, x)
        yq = np.linspace(-20, 80, 50)
        xq = cs(yq)
        return np.column_stack([xq, yq])

    elif name == 'crossing_path':
        y = np.linspace(-20, 80, 40)
        x = 0.3 * y + 10 * np.sin(0.05 * y)
        return np.column_stack([x, y])

    elif name == 'vertical_slalom':
        y = np.linspace(-20, 80, 25)
        x = np.zeros_like(y)
        for i in range(len(y)):
            x[i] = 15 if (i // 4) % 2 == 0 else -15
        cs = CubicSpline(y, x)
        yq = np.linspace(-20, 80, 45)
        xq = cs(yq)
        return np.column_stack([xq, yq])

    elif name == 'lateral_sine':
        y = np.linspace(-20, 80, 50)
        x = 18 * np.sin(0.08 * y)
        return np.column_stack([x, y])

    elif name == 'perpendicular_s':
        y = np.linspace(-20, 80, 40)
        x = 20 * np.sin(0.06 * y)
        return np.column_stack([x, y])

    else:
        raise ValueError(f"Unknown scenario: {name}")


class TargetVehicle:
    """Xe mục tiêu di chuyển theo waypoints"""

    def __init__(self, waypoints, speed):
        self.waypoints = np.array(waypoints)
        self.speed = speed
        self.dt = 0.1

        # Tính cumulative distance
        diffs = np.diff(self.waypoints, axis=0)
        dists = np.sqrt(np.sum(diffs**2, axis=1))
        self.cum_dist = np.concatenate([[0], np.cumsum(dists)])
        self.total_dist = self.cum_dist[-1]

        # State
        self.distance_traveled = 0.0
        self.position = self.waypoints[0].copy()
        self.yaw = np.arctan2(diffs[0, 1], diffs[0, 0])

    def step(self):
        """Di chuyển target 1 bước"""
        self.distance_traveled += self.speed * self.dt

        if self.distance_traveled >= self.total_dist:
            self.distance_traveled = self.total_dist
            self.position = self.waypoints[-1].copy()
            return False  # Hết waypoints

        # Nội suy vị trí
        idx = np.searchsorted(self.cum_dist, self.distance_traveled) - 1
        idx = np.clip(idx, 0, len(self.waypoints) - 2)

        frac = ((self.distance_traveled - self.cum_dist[idx]) /
                (self.cum_dist[idx+1] - self.cum_dist[idx] + 1e-10))
        frac = np.clip(frac, 0, 1)

        self.position = ((1 - frac) * self.waypoints[idx] +
                         frac * self.waypoints[idx + 1])

        # Yaw
        diff = self.waypoints[idx+1] - self.waypoints[idx]
        self.yaw = np.arctan2(diff[1], diff[0])

        return True  # Còn tiếp