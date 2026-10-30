"""
Convergence Reliability Index (CRI)

CRI = w1*phi1 + w2*phi2 + w3*phi3 + w4*phi4
"""

import numpy as np
from vehicle_model import wrap_to_pi


class CRIComputer:
    def __init__(self, solver):
        """
        Parameters:
            solver: NMPCSolver instance
        """
        self.solver = solver

        # Tham số CRI
        self.eps1 = 80.0
        self.eps2 = 1.0
        self.eps3 = 0.08
        self.eps4 = 1.2

        self.w1 = 0.25
        self.w2 = 0.20
        self.w3 = 0.40
        self.w4 = 0.15

    def _predict_cost(self, x0, u, x_ref, last_mv):
        """Tính cost khi giữ u không đổi suốt Np bước"""
        J = 0.0
        x_sim = x0.copy()
        s = self.solver

        du = u - last_mv
        J += du @ s.S @ du
        J += u @ s.R @ u

        for j in range(s.Np):
            x_sim = s.model.step(x_sim, u)
            ref_j = x_ref[min(j+1, len(x_ref)-1)]
            e = x_sim - ref_j
            e[2] = wrap_to_pi(e[2])
            J += e @ s.Q @ e

        return J

    def compute(self, x, u_nmpc, u_fallback, x_ref, last_mv):
        """
        Tính CRI

        Parameters:
            x:          [6,] trạng thái hiện tại
            u_nmpc:     [2,] tín hiệu NMPC
            u_fallback: [2,] tín hiệu fallback (PP hoặc Simple)
            x_ref:      [Np+1, 6] tham chiếu
            last_mv:    [2,] tín hiệu bước trước

        Returns:
            cri: float in [0, 1]
            detail: dict
        """
        s = self.solver
        eps_fd = 1e-4

        # ═══ PHI 1: OPTIMALITY ═══
        grad_J = np.zeros(2)
        for i in range(2):
            u_plus = u_nmpc.copy()
            u_plus[i] += eps_fd
            u_minus = u_nmpc.copy()
            u_minus[i] -= eps_fd
            J_plus = self._predict_cost(x, u_plus, x_ref, last_mv)
            J_minus = self._predict_cost(x, u_minus, x_ref, last_mv)
            grad_J[i] = (J_plus - J_minus) / (2 * eps_fd)

        proj_grad = grad_J.copy()
        for i in range(2):
            if abs(u_nmpc[i] - s.u_min[i]) < 1e-6 and grad_J[i] > 0:
                proj_grad[i] = 0
            if abs(u_nmpc[i] - s.u_max[i]) < 1e-6 and grad_J[i] < 0:
                proj_grad[i] = 0

        r_kkt = np.linalg.norm(proj_grad)
        phi1 = np.exp(-r_kkt / self.eps1)

        # ═══ PHI 2: FEASIBILITY ═══
        v_con = 0.0
        x_sim = x.copy()
        for j in range(min(s.Np, 5)):
            x_sim = s.model.step(x_sim, u_nmpc)
            for idx in range(6):
                if x_sim[idx] < s.x_min[idx]:
                    v_con += (s.x_min[idx] - x_sim[idx])**2
                if x_sim[idx] > s.x_max[idx]:
                    v_con += (x_sim[idx] - s.x_max[idx])**2

        for i in range(2):
            if u_nmpc[i] < s.u_min[i]:
                v_con += (s.u_min[i] - u_nmpc[i])**2
            if u_nmpc[i] > s.u_max[i]:
                v_con += (u_nmpc[i] - s.u_max[i])**2

        v_con = np.sqrt(v_con)
        phi2 = 1.0 if v_con == 0 else np.exp(-v_con / self.eps2)

        # ═══ PHI 3: SUPERIORITY ═══
        J_nmpc = self._predict_cost(x, u_nmpc, x_ref, last_mv)
        J_fb = self._predict_cost(x, u_fallback, x_ref, last_mv)

        if J_fb > 1e-6:
            rho = (J_fb - J_nmpc) / J_fb
        else:
            rho = 0.0

        phi3 = 1.0 / (1.0 + np.exp(-rho / self.eps3))

        # ═══ PHI 4: SMOOTHNESS ═══
        du = u_nmpc - last_mv
        du_max = np.array([3.0, 0.1])
        u_jump = np.sqrt((du[0]/du_max[0])**2 + (du[1]/du_max[1])**2)
        phi4 = np.exp(-u_jump / self.eps4)

        # ═══ TỔNG HỢP ═══
        cri = self.w1*phi1 + self.w2*phi2 + self.w3*phi3 + self.w4*phi4
        cri = np.clip(cri, 0.0, 1.0)

        detail = {
            'phi1': phi1, 'phi2': phi2, 'phi3': phi3, 'phi4': phi4,
            'r_kkt': r_kkt, 'v_con': v_con,
            'improvement_ratio': rho, 'u_jump': u_jump
        }

        return cri, detail