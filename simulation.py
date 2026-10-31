"""
Headless closed-loop simulation (NMPC + Pure Pursuit + CRI switching).

Timing convention for step k (time t_k = k * Ts):
    1. ego state x_k and target state p_k are both at t_k and are logged together
    2. sensing / tracking / control use only information at t_k
    3. both candidate commands pass through the SAME limit policy before CRI,
       so the command CRI scores is the command applied to the plant
    4. ego and target are advanced together to t_{k+1}
"""

from dataclasses import dataclass

import numpy as np

from vehicle_model import DynamicBicycleModel, VehicleParams, wrap_to_pi
from tracker import SimpleTracker
from NMPC import NMPCSolver
from pure_pursuit import PurePursuitController
from cri import CRIComputer
from hysteresis import HysteresisSwitcher
from scenario import get_lateral_trajectory, TargetVehicle
from sensor import SimulatedSensor

PROXIMITY_THRESHOLD = 4.0  # m, centre distance; proximity event, not a contact test


@dataclass
class SimConfig:
    scenario_name: str = 'lateral_zigzag'
    target_speed: float = 17.0
    ego_start: tuple = (-80.0, -20.0)
    max_steps: int = 300
    seed: int = 1
    fov_deg: float = 60.0
    max_range: float = 100.0
    sensor_offset: float = 1.2
    t_pred: float = 1.5
    d_front: float = 8.0
    # Optional low-speed acceleration floor. Disabled by default; when enabled
    # it is applied to every candidate before CRI evaluates them.
    low_speed_accel_floor: float = None
    low_speed_threshold: float = 10.0
    stop_on_proximity: bool = False


def finalize_command(u, params, low_speed_accel_floor=None,
                     speed=0.0, low_speed_threshold=10.0):
    """Single limit policy shared by all candidates and the applied command."""
    u = np.array(u, dtype=float)
    if not np.all(np.isfinite(u)):
        raise ValueError(f"non-finite command {u}")
    if low_speed_accel_floor is not None and speed < low_speed_threshold:
        u[0] = max(u[0], low_speed_accel_floor)
    return np.clip(u, params.u_min, params.u_max)


def _info_value(info, key):
    if isinstance(info, dict) and key in info:
        try:
            return float(info[key])
        except (TypeError, ValueError):
            return np.nan
    return np.nan


def run_simulation(config=None, on_step=None):
    """Run one episode. `on_step(step_data)` may return False to stop early."""
    cfg = config or SimConfig()
    np.random.seed(cfg.seed)

    waypoints = get_lateral_trajectory(cfg.scenario_name)
    target = TargetVehicle(waypoints, cfg.target_speed)
    sensor = SimulatedSensor(max_range=cfg.max_range, fov_deg=cfg.fov_deg,
                             sensor_offset=cfg.sensor_offset)

    params = VehicleParams()
    model = DynamicBicycleModel(params)
    solver = NMPCSolver(params)
    pp = PurePursuitController(L=params.L)
    cri_computer = CRIComputer(solver)
    switcher = HysteresisSwitcher(initial_mode='FALLBACK')
    tracker = SimpleTracker(dt=params.Ts)

    x = np.array([cfg.ego_start[0], cfg.ego_start[1], 0.0, 0.0, 0.0, 0.0])
    last_applied = np.zeros(2)
    nan2 = np.full(2, np.nan)

    keys = ['time', 'ego_state', 'target_pos', 'target_yaw', 'distance',
            'mode', 'cri', 'u_applied', 'u_nmpc', 'u_pp', 'exit_flag',
            'grad_norm', 'solver_error', 'lat_accel_kin', 'lat_accel_dyn',
            'tire_util']
    log = {k: [] for k in keys}
    counts = {'NMPC': 0, 'PP_FALLBACK': 0, 'NO_TRACK': 0}
    proximity_time = None
    end_reason = 'max_steps'

    for step in range(cfg.max_steps):
        t = step * params.Ts
        speed = float(np.hypot(x[3], x[4]))
        distance = float(np.linalg.norm(x[:2] - target.position))

        if proximity_time is None and distance < PROXIMITY_THRESHOLD:
            proximity_time = t

        # Sensing and tracking at t_k
        detection = sensor.detect(x[:2], x[2], target.position,
                                  target.speed, target.yaw)
        tracks = tracker.step([detection] if detection is not None else [])

        intercept_point = None
        cri = 0.0
        u_nmpc, u_pp = nan2.copy(), nan2.copy()
        exit_flag = grad_norm = np.nan
        solver_error = ''

        def policy(u):
            return finalize_command(u, params, cfg.low_speed_accel_floor,
                                    speed, cfg.low_speed_threshold)

        if len(tracks) > 0:
            track = tracks[0]
            track_pos = track['State'][:2]
            track_v = track['State'][2]
            track_yaw = track['State'][3]
            heading = np.array([np.cos(track_yaw), np.sin(track_yaw)])

            pred_pos = track_pos + cfg.t_pred * track_v * heading
            intercept_point = pred_pos + cfg.d_front * heading
            dx = intercept_point[0] - x[0]
            dy = intercept_point[1] - x[1]
            heading_error = abs(wrap_to_pi(np.arctan2(dy, dx) - x[2]))
            desired_speed = pp.compute_desired_speed(
                distance, abs(dy), heading_error)

            u_pp = policy(pp.compute(x, intercept_point, desired_speed))

            x_ref = np.zeros((solver.Np + 1, 6))
            yaw_ref = np.arctan2(dy, dx)
            for k in range(solver.Np + 1):
                a_k = k / solver.Np
                x_ref[k, 0] = x[0] + a_k * dx
                x_ref[k, 1] = x[1] + a_k * dy
                x_ref[k, 2] = wrap_to_pi(
                    x[2] + a_k * wrap_to_pi(yaw_ref - x[2]))
                x_ref[k, 3] = desired_speed

            try:
                mv, info = solver.solve(x, last_applied, x_ref)
                u_nmpc = policy(mv)
                exit_flag = _info_value(info, 'ExitFlag')
                grad_norm = _info_value(info, 'GradNorm')
                cri, _ = cri_computer.compute(
                    x, u_nmpc, u_pp, x_ref, last_applied)
                mode = switcher.decide(cri)
                if mode == 'NMPC':
                    u, current_mode = u_nmpc.copy(), 'NMPC'
                else:
                    u, current_mode = u_pp.copy(), 'PP_FALLBACK'
            except Exception as exc:
                solver_error = f"{type(exc).__name__}: {exc}"
                cri = 0.0
                u, current_mode = u_pp.copy(), 'PP_FALLBACK'
        else:
            u = policy([-0.3 * speed, 0.0])
            current_mode = 'NO_TRACK'

        counts[current_mode] += 1
        last_applied = u.copy()

        log['time'].append(t)
        log['ego_state'].append(x.copy())
        log['target_pos'].append(target.position.copy())
        log['target_yaw'].append(target.yaw)
        log['distance'].append(distance)
        log['mode'].append(current_mode)
        log['cri'].append(cri)
        log['u_applied'].append(u.copy())
        log['u_nmpc'].append(u_nmpc)
        log['u_pp'].append(u_pp)
        log['exit_flag'].append(exit_flag)
        log['grad_norm'].append(grad_norm)
        log['solver_error'].append(solver_error)
        # Kinematic estimate only; not a tyre-force based acceleration.
        log['lat_accel_kin'].append(abs(speed ** 2 * np.tan(u[1]) / params.L))
        log['lat_accel_dyn'].append(abs(model.lateral_accel(x, u)))
        log['tire_util'].append(model.tire_utilization(x, u))

        # Advance ego and target together to t_{k+1}
        x = model.step(x, u)
        target_alive = target.step()

        if cfg.stop_on_proximity and proximity_time is not None:
            end_reason = 'proximity'
            break

        if on_step is not None:
            keep_going = on_step({
                'time': t + params.Ts,
                'ego_pos': x[:2].copy(),
                'ego_yaw': x[2],
                'ego_speed': float(np.hypot(x[3], x[4])),
                'target_pos': target.position.copy(),
                'target_yaw': target.yaw,
                'distance': float(np.linalg.norm(x[:2] - target.position)),
                'intercept_point': intercept_point,
                'cri': cri,
                'mode': current_mode,
                'nlmpc_count': counts['NMPC'],
                'pp_count': counts['PP_FALLBACK'],
                'no_track_count': counts['NO_TRACK'],
                'min_distance': min(log['distance']),
            })
            if keep_going is False:
                end_reason = 'user_stop'
                break

        if not target_alive:
            end_reason = 'target_end'
            break

    result = {k: (np.array(v) if k != 'solver_error' else list(v))
              for k, v in log.items()}
    result['ego_history'] = result['ego_state'][:, :2]
    result['target_history'] = result['target_pos']
    result['counts'] = counts
    result['proximity_time'] = proximity_time
    result['end_reason'] = end_reason
    result['params'] = params
    result['config'] = cfg
    return result
