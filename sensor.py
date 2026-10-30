"""
Simulated Vision Sensor
Thêm nhiễu vào đo lường target
"""

import numpy as np
import math
from vehicle_model import wrap_to_pi


class SimulatedSensor:
    def __init__(self, max_range=100.0, fov_deg=60.0, sensor_offset=1.2,
                 noise_pos=0.5, noise_vel=0.3, noise_heading=0.1):
        self.max_range = max_range
        self.fov = math.radians(fov_deg)          # góc FOV (mặc định 60°)
        self.sensor_offset = sensor_offset        # đặt cảm biến tại đầu xe (m)
        self.noise_pos = noise_pos
        self.noise_vel = noise_vel
        self.noise_heading = noise_heading

    def detect(self, ego_pos, ego_yaw, target_pos, target_speed, target_yaw):
        """
        Mô phỏng detection

        Returns:
            detection: [x, y, v, theta] (global frame, có nhiễu)
                       hoặc None nếu ngoài tầm hoặc ngoài FOV
        """
        # Điểm đặt cảm biến tại đầu xe
        sensor_pos = np.array(ego_pos) + self.sensor_offset * np.array(
            [math.cos(ego_yaw), math.sin(ego_yaw)]
        )

        rel = target_pos - sensor_pos
        dist = np.linalg.norm(rel)
        if dist > self.max_range:
            return None

        # Kiểm tra trong hình nón FOV
        bearing = math.atan2(rel[1], rel[0])
        rel_angle = wrap_to_pi(bearing - ego_yaw)
        if abs(rel_angle) > self.fov / 2:
            return None

        # Thêm nhiễu Gaussian
        x_meas = target_pos[0] + np.random.randn() * self.noise_pos
        y_meas = target_pos[1] + np.random.randn() * self.noise_pos
        v_meas = target_speed + np.random.randn() * self.noise_vel
        theta_meas = target_yaw + np.random.randn() * self.noise_heading

        return np.array([x_meas, y_meas, max(0, v_meas), theta_meas])