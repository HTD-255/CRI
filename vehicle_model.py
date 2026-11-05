"""
Dynamic Bicycle Model - 6 trạng thái
x = [X, Y, psi, vx, vy, r]
u = [ax, delta]
"""

import numpy as np


class VehicleParams:
    """Tham số xe

    Nguồn tham số:
        - m, Iz: gần khớp với ví dụ sedan tham chiếu trong Rajesh Rajamani,
          "Vehicle Dynamics and Control" (Springer, 2012), chương 3,
          trong đó m=1573 kg, Iz=2873 kg*m^2 được mô tả là đại diện cho
          một sedan hành khách. lf, lr và Cf, Cr KHÔNG khớp với giá trị
          tương ứng của nguồn này (Rajamani dùng lf=1.1, lr=1.58,
          Cf=Cr=80000 N/rad) nên không được coi là đã có cùng nguồn gốc;
          cần rà soát/hiệu chỉnh lại Cf, Cr trước khi dùng cho tuyên bố
          định lượng.
        - mu: đối chiếu khoảng giá trị với Michael Taylor, "Implementation
          and Validation of the Fiala Tire Model in Chrono", Technical
          Report TR-2015-13 (2015), dùng Umin=0.9/Umax=1.0 khi kiểm định
          mô hình Fiala; đây là giá trị minh họa cho kiểm định mô hình,
          không phải hệ số ma sát đo cho lốp/mặt đường cụ thể của xe
          đang mô phỏng.
        - h: chiều cao trọng tâm, dùng để tính chuyển tải dọc khi phanh/
          tăng tốc (`normal_loads`). Giả định minh họa nằm trong khoảng
          điển hình của sedan hành khách (~0.5-0.6 m) được nhắc tới
          trong các tài liệu động lực học xe (ví dụ Rajamani 2012, ch.2
          về load transfer); KHÔNG phải giá trị đo cho xe cụ thể, cần
          nguồn riêng nếu dùng cho tuyên bố định lượng.
    """
    def __init__(self):
        self.m = 1575.0       # Khối lượng (kg)
        self.L = 2.5          # Wheelbase (m)
        self.lf = 1.2         # Trọng tâm -> trục trước (m)
        self.lr = 1.3         # Trọng tâm -> trục sau (m)
        self.h = 0.55         # Chiều cao trọng tâm (m) - giả định minh họa, cần rà soát
        self.Iz = 2875.0      # Mô-men quán tính yaw (kg*m^2)
        self.Cf = 19000.0     # Độ cứng góc trượt cầu trước (N/rad) - giả định, cần rà soát
        self.Cr = 33000.0     # Độ cứng góc trượt cầu sau (N/rad) - giả định, cần rà soát
        self.mu = 0.9         # Hệ số ma sát lốp-mặt đường (giả định minh họa, xem Taylor 2015)
        self.g = 9.81         # Gia tốc trọng trường (m/s^2)
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

    @staticmethod
    def _fiala_lateral_force(alpha, cornering_stiffness, max_lateral_force):
        """Tính lực ngang Fiala cho một cầu, với alpha dương tạo lực dương.

        `max_lateral_force` là lực ngang bão hòa khả dụng của cầu đó
        (mu*Fz sau khi đã trừ phần ma sát dùng cho lực dọc theo friction
        ellipse, xem `_max_lateral_friction`).
        """
        if cornering_stiffness <= 0:
            raise ValueError("cornering stiffness must be positive")
        if max_lateral_force <= 0:
            # Không còn ma sát khả dụng (ví dụ Fz=0 hoặc |Fx| đã dùng hết).
            return 0.0

        tan_alpha = np.tan(alpha)
        mu_fz = max_lateral_force
        saturation_angle = 3.0 * mu_fz / cornering_stiffness
        if abs(tan_alpha) >= saturation_angle:
            return mu_fz * np.sign(tan_alpha)

        return (
            cornering_stiffness * tan_alpha
            - cornering_stiffness**2 / (3.0 * mu_fz)
            * abs(tan_alpha) * tan_alpha
            + cornering_stiffness**3 / (27.0 * mu_fz**2)
            * tan_alpha**3
        )

    @staticmethod
    def _max_lateral_friction(friction_coefficient, normal_load,
                              longitudinal_force):
        """Friction ellipse: lực ngang tối đa còn lại sau khi trừ phần ma
        sát đã dùng cho lực dọc `longitudinal_force` tại cùng cầu.
        """
        if friction_coefficient <= 0 or normal_load <= 0:
            return 0.0
        max_total = friction_coefficient * normal_load
        ratio = np.clip(longitudinal_force / max_total, -1.0, 1.0)
        return max_total * np.sqrt(max(0.0, 1.0 - ratio**2))

    def normal_loads(self, ax):
        """Tải pháp tuyến động theo trục, có chuyển tải dọc do `ax`.

        Quy ước: `ax > 0` (tăng tốc) chuyển tải về cầu sau; `ax < 0`
        (phanh) chuyển tải về cầu trước. Công thức chuyển tải chuẩn:
        `Fzf = Fzf_static - m*ax*h/L`, `Fzr = Fzr_static + m*ax*h/L`.
        Tải pháp tuyến được chặn dưới ở 0 (không cho giá trị âm khi
        chuyển tải vượt quá tải tĩnh, trường hợp phi thực tế ở gia tốc
        rất lớn).
        """
        p = self.params
        Fzf_static = p.m * p.g * p.lr / p.L
        Fzr_static = p.m * p.g * p.lf / p.L
        transfer = p.m * ax * p.h / p.L
        Fzf = max(Fzf_static - transfer, 0.0)
        Fzr = max(Fzr_static + transfer, 0.0)
        return Fzf, Fzr

    def _longitudinal_force_split(self, ax, Fzf, Fzr):
        """Phân bổ lực dọc tổng (`m*ax`) cho hai cầu theo tỉ lệ tải pháp
        tuyến ĐỘNG (`Fzf`, `Fzr` sau chuyển tải, từ `normal_loads`), thay
        vì tải tĩnh.

        Đây vẫn là một giả định đơn giản hóa (coi như lực kéo/phanh được
        phân bổ tỉ lệ với tải khả dụng mỗi cầu, không mô hình hệ dẫn động
        cụ thể cầu trước/cầu sau/AWD); cần rà soát lại nếu kịch bản có
        gia tốc dọc lớn và cần độ chính xác cao hơn theo drivetrain thật.
        Dùng tải động thay vì tĩnh để tự nhất quán với ngân sách ma sát
        của `_max_lateral_friction` tại cùng cầu: nếu chia theo tải tĩnh,
        lực dọc yêu cầu tại một cầu có thể vượt quá `mu*Fz` động của
        chính cầu đó (ví dụ cầu trước khi tăng tốc gắt làm giảm tải
        trước), khiến cầu đó bị tính sai là mất toàn bộ độ bám ngang dù
        tổng lực dọc toàn xe vẫn khả thi.
        """
        total_Fz = Fzf + Fzr
        if total_Fz <= 0:
            return 0.0, 0.0
        front_frac = Fzf / total_Fz
        Fx_total = self.params.m * ax
        return Fx_total * front_frac, Fx_total * (1.0 - front_frac)

    def tire_forces(self, vx, vy, r, delta, ax=0.0):
        """Tính lực ngang Fiala tại cầu trước và cầu sau.

        `ax` (mặc định 0) dùng để tính tải pháp tuyến động (chuyển tải
        dọc) và giới hạn ma sát theo friction ellipse kết hợp lực dọc.
        """
        p = self.params

        # Tránh chia cho 0
        vx_safe = max(abs(vx), 0.5) * np.sign(max(vx, 0.5))

        # Góc trượt
        alpha_f = delta - np.arctan2(vy + p.lf * r, vx_safe)
        alpha_r = -np.arctan2(vy - p.lr * r, vx_safe)

        # Tải pháp tuyến động (có chuyển tải dọc) và friction ellipse.
        Fzf, Fzr = self.normal_loads(ax)
        Fxf, Fxr = self._longitudinal_force_split(ax, Fzf, Fzr)
        max_Fyf = self._max_lateral_friction(p.mu, Fzf, Fxf)
        max_Fyr = self._max_lateral_friction(p.mu, Fzr, Fxr)

        Fyf = self._fiala_lateral_force(alpha_f, p.Cf, max_Fyf)
        Fyr = self._fiala_lateral_force(alpha_r, p.Cr, max_Fyr)

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

        # Lực lốp (truyền ax để tính chuyển tải dọc + friction ellipse)
        Fyf, Fyr = self.tire_forces(vx, vy, r, delta, ax)

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