"""
Unscented Kalman Filter

Trạng thái target: s = [x, y, v, theta]
Mô hình: Constant Velocity + Constant Heading
"""

import numpy as np
from vehicle_model import wrap_to_pi


class UKF:
    def __init__(self, x0, P0, Q, R, dt):
        """
        Parameters:
            x0: [4,] trạng thái ban đầu [x, y, v, theta]
            P0: [4,4] covariance ban đầu
            Q:  [4,4] nhiễu quá trình
            R:  [4,4] nhiễu đo lường
            dt: bước thời gian
        """
        self.x = np.array(x0, dtype=float)
        self.P = np.array(P0, dtype=float)
        self.Q = np.array(Q, dtype=float)
        self.R = np.array(R, dtype=float)
        self.dt = dt
        self.n = len(x0)  # n = 4

        # ═══ Tham số UKF ═══
        self.alpha = 1e-3
        self.beta = 2.0
        self.kappa = 0.0
        self.lam = self.alpha**2 * (self.n + self.kappa) - self.n

        # ═══ Trọng số sigma points ═══
        n = self.n
        lam = self.lam
        self.Wm = np.zeros(2 * n + 1)
        self.Wc = np.zeros(2 * n + 1)

        self.Wm[0] = lam / (n + lam)
        self.Wc[0] = lam / (n + lam) + (1 - self.alpha**2 + self.beta)
        for i in range(1, 2 * n + 1):
            self.Wm[i] = 1.0 / (2.0 * (n + lam))
            self.Wc[i] = 1.0 / (2.0 * (n + lam))

        # ═══ Tracker state ═══
        self.age = 0
        self.hits = 1
        self.misses = 0
        self.confirmed = False

    def _sigma_points(self):
        """
        Tạo 2n+1 = 9 sigma points

        chi[0]       = x_hat                     (trung tâm)
        chi[i]       = x_hat + L[:,i-1]          i = 1..n
        chi[n+i]     = x_hat - L[:,i-1]          i = 1..n
        """
        n = self.n
        scaling = (n + self.lam)

        # Cholesky an toàn
        try:
            L = np.linalg.cholesky(scaling * self.P)
        except np.linalg.LinAlgError:
            eigvals = np.linalg.eigvalsh(self.P)
            min_eig = np.min(eigvals)
            self.P += (-min_eig + 1e-6) * np.eye(n)
            L = np.linalg.cholesky(scaling * self.P)

        sigma = np.zeros((2 * n + 1, n))
        sigma[0] = self.x

        for i in range(n):
            sigma[i + 1] = self.x + L[:, i]
            sigma[n + i + 1] = self.x - L[:, i]

        return sigma

    def _process_model(self, s):
        """
        Mô hình CVCH:
            x_new     = x + dt * v * cos(theta)
            y_new     = y + dt * v * sin(theta)
            v_new     = v
            theta_new = theta
        """
        x, y, v, theta = s
        return np.array([
            x + self.dt * v * np.cos(theta),
            y + self.dt * v * np.sin(theta),
            v,
            theta
        ])

    def _measurement_model(self, s):
        """Đo trực tiếp: z = s"""
        return s.copy()

    def predict(self):
        """
        BƯỚC DỰ ĐOÁN:
        1. Tạo sigma points
        2. Lan truyền qua process model
        3. Tính x_pred, P_pred
        """
        n = self.n
        sigma = self._sigma_points()

        # Lan truyền
        sigma_pred = np.zeros_like(sigma)
        for i in range(2 * n + 1):
            sigma_pred[i] = self._process_model(sigma[i])

        # Trung bình dự đoán
        x_pred = np.zeros(n)
        for i in range(2 * n + 1):
            x_pred += self.Wm[i] * sigma_pred[i]

        # Covariance dự đoán
        P_pred = np.zeros((n, n))
        for i in range(2 * n + 1):
            d = sigma_pred[i] - x_pred
            d[3] = wrap_to_pi(d[3])  # wrap theta
            P_pred += self.Wc[i] * np.outer(d, d)
        P_pred += self.Q

        self.x = x_pred
        self.P = P_pred

    def update(self, z):
        """
        BƯỚC CẬP NHẬT:
        1. Tạo sigma points từ (x_pred, P_pred)
        2. Tính z_hat, Pzz, Pxz
        3. Tính Kalman Gain K
        4. Cập nhật x, P
        """
        z = np.array(z, dtype=float)
        n = self.n
        nz = len(z)

        sigma = self._sigma_points()

        # Lan truyền qua measurement model
        sigma_z = np.zeros((2 * n + 1, nz))
        for i in range(2 * n + 1):
            sigma_z[i] = self._measurement_model(sigma[i])

        # Trung bình đo lường dự đoán
        z_hat = np.zeros(nz)
        for i in range(2 * n + 1):
            z_hat += self.Wm[i] * sigma_z[i]

        # Innovation covariance Pzz
        Pzz = np.zeros((nz, nz))
        for i in range(2 * n + 1):
            dz = sigma_z[i] - z_hat
            if nz >= 4:
                dz[3] = wrap_to_pi(dz[3])
            Pzz += self.Wc[i] * np.outer(dz, dz)
        Pzz += self.R

        # Cross-covariance Pxz
        Pxz = np.zeros((n, nz))
        for i in range(2 * n + 1):
            dx = sigma[i] - self.x
            dx[3] = wrap_to_pi(dx[3])
            dz = sigma_z[i] - z_hat
            if nz >= 4:
                dz[3] = wrap_to_pi(dz[3])
            Pxz += self.Wc[i] * np.outer(dx, dz)

        # Kalman Gain
        K = Pxz @ np.linalg.inv(Pzz)

        # Innovation
        nu = z - z_hat
        if nz >= 4:
            nu[3] = wrap_to_pi(nu[3])

        # Cập nhật
        self.x = self.x + K @ nu
        self.x[3] = wrap_to_pi(self.x[3])

        self.P = self.P - K @ Pzz @ K.T
        self.P = (self.P + self.P.T) / 2.0  # đảm bảo đối xứng

        # Tracker state
        self.hits += 1
        self.misses = 0

    def missed(self):
        """Không có đo lường"""
        self.misses += 1

    def update_age(self):
        self.age += 1
        if self.hits >= 2 and self.age >= 2:
            self.confirmed = True

    def should_delete(self):
        return self.misses >= 2