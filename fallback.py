"""
Simple Proportional Fallback Controller

delta = 0.3 * delta_psi
a = 2.0 * (v_desired - v_current)
"""

import numpy as np
from vehicle_model import wrap_to_pi


class SimpleFallbackController:
    def __init__(self):
        self.K_yaw = 0.3
        self.Ka = 2.0
        self.delta_min = -0.45
        self.delta_max = 0.45
        self.a_min = -4.0
        self.a_max = 8.0

    def compute(self, x, intercept_point, desired_speed):
        """
        Tính tín hiệu Simple Proportional

        Parameters:
            x: [6,] trạng thái ego
            intercept_point: [2,] vị trí đích
            desired_speed: float

        Returns:
            u_simple: [2,] [a, delta]
        """
        dx = intercept_point[0] - x[0]
        dy = intercept_point[1] - x[1]

        target_yaw = np.arctan2(dy, dx)
        yaw_diff = wrap_to_pi(target_yaw - x[2])

        # Góc lái đơn giản
        delta = self.K_yaw * yaw_diff
        delta = np.clip(delta, self.delta_min, self.delta_max)

        # Gia tốc đơn giản
        current_speed = np.sqrt(x[3]**2 + x[4]**2)
        a = self.Ka * (desired_speed - current_speed)
        a = np.clip(a, self.a_min, self.a_max)

        return np.array([a, delta])