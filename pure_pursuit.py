"""
Pure Pursuit Controller

delta_PP = arctan(2 * L * sin(alpha) / d)
a_PP = Ka * (v_desired - v_current)
"""

import numpy as np
from vehicle_model import wrap_to_pi


class PurePursuitController:
    def __init__(self, L=2.5):
        self.L = L
        self.Ka = 3.5
        self.delta_min = -0.45
        self.delta_max = 0.45
        self.a_min = -4.0
        self.a_max = 8.0

    def compute(self, x, intercept_point, desired_speed):
        """
        Tính tín hiệu Pure Pursuit

        Parameters:
            x: [6,] trạng thái ego [X, Y, psi, vx, vy, r]
            intercept_point: [2,] vị trí đích [x, y]
            desired_speed: float tốc độ mong muốn

        Returns:
            u_pp: [2,] [a_PP, delta_PP]
        """
        dx = intercept_point[0] - x[0]
        dy = intercept_point[1] - x[1]

        # Góc nhìn
        alpha = np.arctan2(dy, dx) - x[2]
        alpha = wrap_to_pi(alpha)

        # Khoảng cách
        d = np.sqrt(dx**2 + dy**2)

        # Góc lái PP
        delta_pp = np.arctan2(2 * self.L * np.sin(alpha), d + 1e-6)
        delta_pp = np.clip(delta_pp, self.delta_min, self.delta_max)

        # Gia tốc PP
        current_speed = np.sqrt(x[3]**2 + x[4]**2)
        a_pp = self.Ka * (desired_speed - current_speed)
        a_pp = np.clip(a_pp, self.a_min, self.a_max)

        return np.array([a_pp, delta_pp])

    @staticmethod
    def compute_desired_speed(cam_to_rear, lateral_offset, heading_error):
        """Chiến lược tốc độ thích nghi"""
        if cam_to_rear > 20:
            speed = 22.0
        elif cam_to_rear > 10:
            if lateral_offset > 15 or heading_error > np.deg2rad(45):
                speed = 16.0
            else:
                speed = 20.0
        elif cam_to_rear > 5:
            if lateral_offset > 10 or heading_error > np.deg2rad(30):
                speed = 14.0
            else:
                speed = 18.0
        else:
            speed = 15.0

        return max(speed, 12.0)