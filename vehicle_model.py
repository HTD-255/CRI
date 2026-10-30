"""
Dynamic Bicycle Model - 6 trạng thái
x = [X, Y, psi, vx, vy, r]
u = [ax, delta]
"""

import numpy as np


class VehicleParams:
    """Tham số xe"""
    def __init__(self):
        self.m = 1575.0       # Khối lượng (kg)
        self.L = 2.5          # Wheelbase (m)
        self.lf = 1.2         # Trọng tâm -> trục trước (m)
        self.lr = 1.3         # Trọng tâm -> trục sau (m)
        self.Iz = 2875.0      # Mô-men quán tính yaw (kg*m^2)
        self.Cf = 19000.0     # Độ cứng lốp trước (N/rad)
        self.Cr = 33000.0     # Độ cứng lốp sau (N/rad)
        self.Ts = 0.1         # Chu kỳ lấy mẫu (s)

        # Ràng buộc điều khiển
        self.ax_min = -5.0
        self.ax_max = 8.0
        self.delta_min = -0.45
        self.delta_max = 0.45

        # Ràng buộc trạng thái
        self.X_min, self.X_max = -200.0, 200.0
        self.Y_min, self.Y_max = -200.0, 200.0
        self.psi_min, self.psi_max = -np.pi, np.pi
        self.vx_min, self.vx_max = 0.0, 35.0
        self.vy_min, self.vy_max = -5.0, 5.0
        self.r_min, self.r_max = -1.2, 1.2

    @property
    def u_min(self):
        return np.array([self.ax_min, self.delta_min])

    @property
    def u_max(self):
        return np.array([self.ax_max, self.delta_max])

    @property
    def x_min(self):
        return np.array([self.X_min, self.Y_min, self.psi_min,
                         self.vx_min, self.vy_min, self.r_min])

    @property
    def x_max(self):
        return np.array([self.X_max, self.Y_max, self.psi_max,
                         self.vx_max, self.vy_max, self.r_max])


def wrap_to_pi(angle):
    """Wrap góc vào [-pi, pi]"""
    return (angle + np.pi) % (2 * np.pi) - np.pi


class DynamicBicycleModel:
    """
    Mô hình Dynamic Bicycle 6 trạng thái

    Phương trình:
        dX/dt   = vx*cos(psi) - vy*sin(psi)
        dY/dt   = vx*sin(psi) + vy*cos(psi)
        dpsi/dt = r
        dvx/dt  = ax + vy*r
        dvy/dt  = (Fyf*cos(delta) + Fyr)/m - vx*r
        dr/dt   = (lf*Fyf*cos(delta) - lr*Fyr)/Iz
    """

    def __init__(self, params=None):
        self.params = params if params else VehicleParams()

    def tire_forces(self, vx, vy, r, delta):
        """Tính lực ngang lốp"""
        p = self.params

        # Tránh chia cho 0
        vx_safe = max(abs(vx), 0.5) * np.sign(max(vx, 0.5))

        # Góc trượt
        alpha_f = delta - np.arctan2(vy + p.lf * r, vx_safe)
        alpha_r = -np.arctan2(vy - p.lr * r, vx_safe)

        # Lực ngang (mô hình tuyến tính)
        Fyf = p.Cf * alpha_f
        Fyr = p.Cr * alpha_r

        return Fyf, Fyr

    def continuous_dynamics(self, x, u):
        """
        Tính đạo hàm dx/dt

        Parameters:
            x: [X, Y, psi, vx, vy, r] - trạng thái
            u: [ax, delta] - điều khiển

        Returns:
            dx_dt: [dX, dY, dpsi, dvx, dvy, dr]
        """
        X, Y, psi, vx, vy, r = x
        ax, delta = u
        p = self.params

        # Lực lốp
        Fyf, Fyr = self.tire_forces(vx, vy, r, delta)

        # 6 phương trình
        dX = vx * np.cos(psi) - vy * np.sin(psi)
        dY = vx * np.sin(psi) + vy * np.cos(psi)
        dpsi = r
        dvx = ax + vy * r
        dvy = (Fyf * np.cos(delta) + Fyr) / p.m - vx * r
        dr = (p.lf * Fyf * np.cos(delta) - p.lr * Fyr) / p.Iz

        return np.array([dX, dY, dpsi, dvx, dvy, dr])

    def step(self, x, u):
        """
        Euler discretization: x(k+1) = x(k) + Ts * f(x(k), u(k))

        Parameters:
            x: [6,] trạng thái hiện tại
            u: [2,] điều khiển

        Returns:
            x_next: [6,] trạng thái tiếp theo
        """
        x = np.array(x, dtype=float)
        u = np.array(u, dtype=float)

        dx_dt = self.continuous_dynamics(x, u)
        x_next = x + self.params.Ts * dx_dt

        # Wrap yaw
        x_next[2] = wrap_to_pi(x_next[2])

        return x_next