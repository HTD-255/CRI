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

        # Mô hình lốp: 'saturated' (giới hạn vòng tròn ma sát) hoặc 'linear'
        self.tire_model = 'saturated'
        self.mu = 0.8         # Hệ số bám đường khô (nhựa đường)
        self.g = 9.81         # Gia tốc trọng trường (m/s^2)
        self.long_grip_frac = 0.9  # Lực dọc tối đa theo tỷ lệ mu*Fz (chừa lực ngang)

        # Magic Formula 6.1 (lực ngang thuần), tham khảo example.tir của
        # JyNing04/Pacejka-tire-model (Apache-2.0); chỉ dùng khi tire_model='pacejka'
        self.h_cg = 0.55           # Độ cao trọng tâm (m), giả định cho sedan
        self.Fz0_tire = self.m * self.g / 4.0   # Tải danh định mỗi lốp (N)
        self.PCY1 = 2.0            # Hệ số hình dạng C (giá trị của example.tir)
        self.PDY1_ratio = 1.0      # Đỉnh ma sát = PDY1_ratio * mu
        self.PDY2 = -0.05          # Giảm ma sát theo tải
        self.PEY1 = -0.8           # Độ cong E tại tải danh định
        self.PEY2 = -0.6           # Độ cong E theo tải
        self.PEY3 = 0.1            # Bất đối xứng E theo dấu góc trượt
        self.PKY2 = 2.0            # Tải tại đó độ cứng đạt cực đại (theo Fz0)
        self.PKY4 = 2.0            # Độ cong độ cứng theo tải

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

    def effective_ax(self, ax):
        """Gia tốc dọc thực tế: không vượt quá khả năng bám dọc cho phép"""
        p = self.params
        if p.tire_model == 'linear':
            return ax
        lim = p.long_grip_frac * p.mu * p.g
        return float(np.clip(ax, -lim, lim))

    def axle_force_limits(self, ax):
        """
        Lực ngang tối đa mỗi trục theo vòng tròn ma sát.
        Tải trọng tĩnh theo phân bố khối lượng; lực dọc chia theo tải trọng
        trục nên mỗi trục dùng cùng tỷ lệ bám cho lực dọc.
        """
        p = self.params
        Fzf = p.m * p.g * p.lr / p.L
        Fzr = p.m * p.g * p.lf / p.L
        a = self.effective_ax(ax)
        scale = np.sqrt(max(p.mu ** 2 - (a / p.g) ** 2, 0.0))
        return max(Fzf * scale, 1.0), max(Fzr * scale, 1.0)

    def slip_angles(self, vx, vy, r, delta):
        p = self.params
        # Tránh chia cho 0
        vx_safe = max(abs(vx), 0.5) * np.sign(max(vx, 0.5))
        alpha_f = delta - np.arctan2(vy + p.lf * r, vx_safe)
        alpha_r = -np.arctan2(vy - p.lr * r, vx_safe)
        return alpha_f, alpha_r

    def tire_forces(self, vx, vy, r, delta, ax=0.0):
        """
        Lực ngang lốp. Chế độ 'saturated' dùng tanh: độ dốc tại gốc bằng
        độ cứng Cf/Cr và bão hòa tại giới hạn vòng tròn ma sát.
        """
        p = self.params
        alpha_f, alpha_r = self.slip_angles(vx, vy, r, delta)

        if p.tire_model == 'linear':
            return p.Cf * alpha_f, p.Cr * alpha_r

        if p.tire_model == 'pacejka':
            Fzf, Fzr = self.axle_loads(ax)
            Fzf0 = p.m * p.g * p.lr / p.L
            Fzr0 = p.m * p.g * p.lf / p.L
            scale = self.friction_circle_scale(ax)
            return (self._mf_axle(alpha_f, Fzf, Fzf0, p.Cf, scale),
                    self._mf_axle(alpha_r, Fzr, Fzr0, p.Cr, scale))

        Ff_max, Fr_max = self.axle_force_limits(ax)
        Fyf = Ff_max * np.tanh(p.Cf * alpha_f / Ff_max)
        Fyr = Fr_max * np.tanh(p.Cr * alpha_r / Fr_max)
        return Fyf, Fyr

    def friction_circle_scale(self, ax):
        p = self.params
        a = self.effective_ax(ax)
        return np.sqrt(max(p.mu ** 2 - (a / p.g) ** 2, 0.0)) / p.mu

    def axle_loads(self, ax):
        """Tải trục: tĩnh + chuyển tải dọc do gia tốc (chỉ chế độ pacejka)"""
        p = self.params
        Fzf = p.m * p.g * p.lr / p.L
        Fzr = p.m * p.g * p.lf / p.L
        if p.tire_model == 'pacejka':
            dF = p.m * self.effective_ax(ax) * p.h_cg / p.L
            Fzf, Fzr = Fzf - dF, Fzr + dF
        return max(Fzf, 1.0), max(Fzr, 1.0)

    def _mf_stiffness(self, Fz_tire):
        """Độ cứng góc trượt theo tải (dạng MF 6.1: sin(PKY4*atan(Fz/(PKY2*Fz0))))"""
        p = self.params
        return np.sin(p.PKY4 * np.arctan(Fz_tire / (p.PKY2 * p.Fz0_tire)))

    def _mf_axle(self, alpha, Fz_axle, Fz_axle_static, C_axle, scale):
        """
        Magic Formula lực ngang thuần (MF 6.1, không camber/áp suất) cho một
        trục gồm 2 lốp. Độ cứng ở tải tĩnh được chuẩn hóa về Cf/Cr để khớp mô
        hình dự đoán ở vùng góc trượt nhỏ; hình dạng đỉnh/bão hòa theo MF.
        """
        p = self.params
        Fz = Fz_axle / 2.0
        dfz = (Fz - p.Fz0_tire) / p.Fz0_tire
        mu = (p.PDY1_ratio * p.mu + p.PDY2 * dfz) * scale
        D = max(mu * Fz, 1e-3)
        Fz_static = Fz_axle_static / 2.0
        K_static = C_axle / 2.0
        K = K_static * self._mf_stiffness(Fz) / self._mf_stiffness(Fz_static)
        Cy = p.PCY1
        By = K / (Cy * D)
        Ey = min((p.PEY1 + p.PEY2 * dfz) * (1.0 - p.PEY3 * np.sign(alpha)), 1.0)
        x = By * alpha
        return 2.0 * D * np.sin(Cy * np.arctan(x - Ey * (x - np.arctan(x))))

    def tire_utilization(self, x, u):
        """
        Mức sử dụng độ bám: max theo trục của |lực tuyến tính yêu cầu| /
        lực ngang tối đa. > 1 nghĩa là lốp đã bão hòa (lệnh vượt khả năng).
        """
        p = self.params
        _, _, _, vx, vy, r = x
        ax, delta = u
        alpha_f, alpha_r = self.slip_angles(vx, vy, r, delta)
        Ff_max, Fr_max = self.axle_force_limits(ax)
        return max(abs(p.Cf * alpha_f) / Ff_max, abs(p.Cr * alpha_r) / Fr_max)

    def lateral_accel(self, x, u):
        """Gia tốc ngang thân xe tính từ lực lốp (m/s^2)"""
        _, _, _, vx, vy, r = x
        ax, delta = u
        Fyf, Fyr = self.tire_forces(vx, vy, r, delta, ax)
        return (Fyf * np.cos(delta) + Fyr) / self.params.m

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
        Fyf, Fyr = self.tire_forces(vx, vy, r, delta, ax)
        ax = self.effective_ax(ax)

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