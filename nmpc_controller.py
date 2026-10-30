"""
nmpc_controller.py — Nonlinear MPC steering controller.

This wrapper exposes the project-level ``NMPCSolver`` through the common
``BaseController`` interface so it can be tested alongside LR / PID / RLS /
MPC.  The solver itself optimises the 2D control vector ``[ax, delta]``;
the controller publishes the steering component as the steering command and
keeps the acceleration term in the telemetry extras.
"""

from typing import Any, Dict, Optional, Tuple

import numpy as np

from NMPC import NMPCSolver
from src.controllers.base_controller import BaseController
from src.utils.config import SENSOR_DT, STEERING_GAIN
from vehicle_model import VehicleParams, wrap_to_pi


class NMPCController(BaseController):
    """Nonlinear MPC controller built on top of ``NMPCSolver``.

    The controller uses a simple straight-ahead reference trajectory because
    the offline unit tests only provide IMU-like sensor signals.  In the full
    project this class can be extended with a richer reference builder using
    target position and heading information.
    """

    def __init__(
        self,
        params: Optional[VehicleParams] = None,
        target_speed: float = 2.0,
        dt: float = SENSOR_DT,
        gain: float = STEERING_GAIN,
        **kwargs: Any,
    ) -> None:
        kwargs.setdefault("use_gyro", True)
        kwargs.setdefault("use_euler", True)
        kwargs.setdefault("use_accel", False)
        super().__init__(name="NMPC", dt=dt, gain=gain, **kwargs)

        self.params = params if params is not None else VehicleParams()
        self.solver = NMPCSolver(self.params)
        self.solver.max_iter = 3
        self.target_speed = target_speed

        self._last_u = np.zeros(2, dtype=float)
        self._target_yaw: Optional[float] = None

    def compute_steering(
        self,
        gyro_z: float,
        euler_yaw: float,
        accel_y: float,
        features: np.ndarray,
    ) -> Tuple[float, Dict[str, Any]]:
        """Solve NMPC and return the steering component as raw output."""
        yaw_rad = np.deg2rad(euler_yaw)
        yaw_rate_rad = np.deg2rad(gyro_z)

        if self._target_yaw is None:
            self._target_yaw = wrap_to_pi(yaw_rad)

        heading_ref = wrap_to_pi(self._target_yaw)
        x0 = np.array([
            0.0,
            0.0,
            wrap_to_pi(yaw_rad),
            max(self.target_speed, 0.5),
            0.0,
            yaw_rate_rad,
        ], dtype=float)

        x_ref = self._build_reference(x0, heading_ref)
        u_opt, info = self.solver.solve(x0, self._last_u, x_ref)
        self._last_u = u_opt.copy()

        extras = {
            "nmpc_ax": float(u_opt[0]),
            "nmpc_delta": float(u_opt[1]),
            "nmpc_cost": float(info["Cost"]),
            "nmpc_grad_norm": float(info["GradNorm"]),
            "nmpc_iterations": int(info["Iterations"]),
            "nmpc_exit_flag": int(info["ExitFlag"]),
            "nmpc_state": x0.tolist(),
        }
        return float(u_opt[1]), extras

    def _build_reference(self, x0: np.ndarray, heading_ref: float) -> np.ndarray:
        """Build a short straight-line reference trajectory for NMPC."""
        x_ref = np.zeros((self.solver.Np + 1, 6), dtype=float)
        vx_ref = float(max(self.target_speed, 0.5))

        for k in range(self.solver.Np + 1):
            travel = k * self.params.Ts * vx_ref
            x_ref[k, 0] = x0[0] + travel * np.cos(heading_ref)
            x_ref[k, 1] = x0[1] + travel * np.sin(heading_ref)
            x_ref[k, 2] = heading_ref
            x_ref[k, 3] = vx_ref
            x_ref[k, 4] = 0.0
            x_ref[k, 5] = 0.0

        return x_ref

    def _reset_internals(self) -> None:
        self._last_u[:] = 0.0
        self._target_yaw = None