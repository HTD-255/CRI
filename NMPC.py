"""
NMPC Solver - Projected Gradient Descent

min  J(U) = J_track + J_control + J_smooth
s.t. u_min <= u <= u_max
"""

import numpy as np
from vehicle_model import DynamicBicycleModel, VehicleParams, wrap_to_pi


class NMPCSolver:
    def __init__(self, params=None):
        self.params = params if params else VehicleParams()
        self.model = DynamicBicycleModel(self.params)

        # Horizons
        self.Np = 8
        self.Nc = 2
        self.Ts = self.params.Ts

        # Trọng số
        self.Q = np.diag([2.0, 3.0, 1.0, 2.5, 1.0, 0.8])
        self.R = np.diag([0.1, 0.5])
        self.S = np.diag([1.0, 12.0])

        # Ràng buộc
        self.u_min = self.params.u_min
        self.u_max = self.params.u_max
        self.x_min = self.params.x_min
        self.x_max = self.params.x_max

        # Solver options
        self.max_iter = 5
        self.step_size = 0.01
        self.tol = 1e-4
        self.eps_fd = 1e-5

    def cost_function(self, x0, U, u_prev, x_ref):
        """
        Tính J(U) bằng forward simulation

        Parameters:
            x0:    [6,] trạng thái hiện tại
            U:     [2*Nc,] biến quyết định [ax0, d0, ax1, d1]
            u_prev:[2,] điều khiển bước trước
            x_ref: [Np+1, 6] quỹ đạo tham chiếu
        """
        J = 0.0
        x_k = x0.copy()
        u_km1 = u_prev.copy()

        for k in range(self.Np):
            # Chọn u cho bước k
            if k < self.Nc:
                u_k = U[2*k : 2*k+2]
            else:
                u_k = U[2*(self.Nc-1) : 2*self.Nc]

            # Mô phỏng
            x_k = self.model.step(x_k, u_k)

            # Tracking error
            ref_k = x_ref[min(k+1, len(x_ref)-1)]
            e = x_k - ref_k
            e[2] = wrap_to_pi(e[2])
            J += e @ self.Q @ e

            # Control effort
            J += u_k @ self.R @ u_k

            # Control rate
            if k < self.Nc:
                du = u_k - u_km1
                J += du @ self.S @ du
                u_km1 = u_k.copy()

        return J

    def cost_and_gradient(self, x0, U, u_prev, x_ref):
        """
        Tính cost và gradient bằng central finite difference

        grad[i] = (J(U+eps*ei) - J(U-eps*ei)) / (2*eps)
        """
        J = self.cost_function(x0, U, u_prev, x_ref)

        nU = len(U)
        grad = np.zeros(nU)

        for i in range(nU):
            U_plus = U.copy()
            U_plus[i] += self.eps_fd
            J_plus = self.cost_function(x0, U_plus, u_prev, x_ref)

            U_minus = U.copy()
            U_minus[i] -= self.eps_fd
            J_minus = self.cost_function(x0, U_minus, u_prev, x_ref)

            grad[i] = (J_plus - J_minus) / (2 * self.eps_fd)

        return J, grad

    def project(self, U):
        """Clip U vào ràng buộc box"""
        for k in range(self.Nc):
            idx = slice(2*k, 2*k+2)
            U[idx] = np.clip(U[idx], self.u_min, self.u_max)
        return U

    def solve(self, x0, u_prev, x_ref):
        """
        Giải bài toán NMPC

        Parameters:
            x0:     [6,] trạng thái hiện tại
            u_prev: [2,] điều khiển bước trước
            x_ref:  [Np+1, 6] tham chiếu

        Returns:
            u_opt:  [2,] điều khiển tối ưu
            info:   dict {Cost, GradNorm, ExitFlag, Iterations}
        """
        x0 = np.array(x0, dtype=float)
        u_prev = np.array(u_prev, dtype=float)

        # Warm start
        U = np.tile(u_prev, self.Nc)

        best_cost = np.inf
        best_U = U.copy()
        converged = False

        for iteration in range(self.max_iter):
            # Cost và gradient
            J, grad = self.cost_and_gradient(x0, U, u_prev, x_ref)

            # Lưu nghiệm tốt nhất
            if J < best_cost:
                best_cost = J
                best_U = U.copy()

            # Kiểm tra hội tụ
            grad_norm = np.linalg.norm(grad)
            if grad_norm < self.tol:
                converged = True
                break

            # Armijo backtracking
            alpha = self.step_size
            for _ in range(10):
                U_new = U - alpha * grad
                U_new = self.project(U_new)
                J_new = self.cost_function(x0, U_new, u_prev, x_ref)

                if J_new < J - 1e-4 * alpha * (grad @ grad):
                    break
                alpha *= 0.5

            U = U_new

        # Dùng nghiệm tốt nhất
        U = best_U
        u_opt = U[:2].copy()
        u_opt = np.clip(u_opt, self.u_min, self.u_max)

        # Projected gradient cho CRI
        _, final_grad = self.cost_and_gradient(x0, U, u_prev, x_ref)
        proj_grad = final_grad[:2].copy()
        for i in range(2):
            if abs(u_opt[i] - self.u_min[i]) < 1e-6 and proj_grad[i] > 0:
                proj_grad[i] = 0
            if abs(u_opt[i] - self.u_max[i]) < 1e-6 and proj_grad[i] < 0:
                proj_grad[i] = 0

        info = {
            'Cost': best_cost,
            'GradNorm': np.linalg.norm(proj_grad),
            'Iterations': min(iteration + 1, self.max_iter),
            'ExitFlag': 1 if converged else 0
        }

        return u_opt, info