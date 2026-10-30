"""
Main: NLMPC + Pure Pursuit (Pygame Animation)
"""

import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')  # không hiện matplotlib window, chỉ save file
import matplotlib.pyplot as plt

from scenario import get_lateral_trajectory
from simulation import SimConfig, run_simulation, PROXIMITY_THRESHOLD
from metric import calculate_pei


def main(headless=False):
    cfg = SimConfig()
    scenario_name = cfg.scenario_name

    print(f"\n{'='*60}")
    print(f"  NLMPC + PURE PURSUIT | {scenario_name.upper()}")
    print(f"{'='*60}\n")

    animator = None
    if not headless:
        from animator import PygameAnimator
        animator = PygameAnimator(get_lateral_trajectory(scenario_name),
                                  np.array(cfg.ego_start))

    on_step = animator.update if animator is not None else None
    log = run_simulation(cfg, on_step=on_step)

    if animator is not None:
        animator.wait_for_close()

    ego_history = log['ego_history']
    target_history = log['target_history']
    time_history = log['time']
    distance_history = log['distance']
    ego_state = log['ego_state']
    speed_history = np.hypot(ego_state[:, 3], ego_state[:, 4])
    control_history = log['u_applied']
    cri_history = log['cri']
    lat_accel_history = log['lat_accel_kin']
    nlmpc_count = log['counts']['NMPC']
    pp_count = log['counts']['PP_FALLBACK']
    no_track_count = log['counts']['NO_TRACK']

    if log['proximity_time'] is not None:
        print(f">>> PROXIMITY EVENT (d < {PROXIMITY_THRESHOLD:.0f} m) "
              f"at t={log['proximity_time']:.1f}s")
    errors = [e for e in log['solver_error'] if e]
    if errors:
        print(f">>> Solver exceptions: {len(errors)} (first: {errors[0]})")

    pei = calculate_pei(ego_history, target_history,
                        distance_history, time_history)
    total = nlmpc_count + pp_count + no_track_count

    print(f"\n{'='*60}")
    print(f"  RESULTS: NLMPC + PP | {scenario_name.upper()}")
    print(f"{'='*60}")
    print(f"  NMPC     : {nlmpc_count:4d} ({nlmpc_count/max(total,1)*100:.1f}%)")
    print(f"  PP FB    : {pp_count:4d} ({pp_count/max(total,1)*100:.1f}%)")
    print(f"  No Track : {no_track_count:4d}")
    print(f"  Min dist : {np.min(distance_history):.2f} m")
    print(f"  PEI      : {pei['overall_score']:.3f}")
    print(f"  CRI mean : {np.mean(cri_history):.3f}")
    print(f"{'='*60}")

    # ═══ MATPLOTLIB SAVE (không hiện cửa sổ) ═══
    fig, axes = plt.subplots(2, 4, figsize=(18, 9))
    fig.suptitle(f'NLMPC + PP | {scenario_name.upper()}', fontsize=14)

    axes[0,0].plot(ego_history[:,0], ego_history[:,1], 'b-', lw=2)
    axes[0,0].plot(target_history[:,0], target_history[:,1], 'r--', lw=1.5)
    axes[0,0].set_title('Trajectories'); axes[0,0].grid(); axes[0,0].set_aspect('equal')

    axes[0,1].plot(time_history, distance_history, 'k-', lw=2)
    axes[0,1].axhline(15, color='r', ls='--'); axes[0,1].axhline(5, color='g', ls='--')
    axes[0,1].set_title('Distance'); axes[0,1].grid()

    axes[0,2].plot(time_history, speed_history, 'b-', lw=2)
    axes[0,2].set_title('Speed'); axes[0,2].grid()

    t_cri = time_history[:len(cri_history)]
    axes[0,3].plot(t_cri, cri_history, 'b-', lw=1.5)
    axes[0,3].axhline(0.65, color='g', ls='--'); axes[0,3].axhline(0.45, color='r', ls='--')
    axes[0,3].set_title('CRI'); axes[0,3].set_ylim([0, 1.05]); axes[0,3].grid()

    t_ctrl = time_history[:len(control_history)]
    axes[1,0].plot(t_ctrl, control_history[:,0], 'r-', lw=1.5)
    axes[1,0].set_title('Acceleration'); axes[1,0].grid()

    axes[1,1].plot(t_ctrl, np.rad2deg(control_history[:,1]), 'm-', lw=1.5)
    axes[1,1].set_title('Steering'); axes[1,1].grid()

    t_lat = time_history[:len(lat_accel_history)]
    axes[1,2].plot(t_lat, lat_accel_history, 'b-', lw=1.5)
    axes[1,2].axhline(8, color='r', ls='--')
    axes[1,2].set_title('Lateral Accel'); axes[1,2].grid()

    if total > 0:
        labels, sizes, colors = [], [], []
        if nlmpc_count > 0: labels.append(f'NMPC ({nlmpc_count})'); sizes.append(nlmpc_count); colors.append('#2ecc71')
        if pp_count > 0: labels.append(f'PP ({pp_count})'); sizes.append(pp_count); colors.append('#e67e22')
        if no_track_count > 0: labels.append(f'NoTrack ({no_track_count})'); sizes.append(no_track_count); colors.append('#e74c3c')
        axes[1,3].pie(sizes, labels=labels, colors=colors)
        axes[1,3].set_title('Mode')

    plt.tight_layout()
    plt.savefig(f'results_nlmpc_pp_{scenario_name}.png', dpi=150)
    print(f"Saved: results_nlmpc_pp_{scenario_name}.png")

    np.savez(f'results_nlmpc_pp_{scenario_name}.npz',
             ego_history=ego_history, target_history=target_history,
             time_history=time_history, distance_history=distance_history,
             speed_history=speed_history, control_history=control_history,
             cri_history=cri_history, lat_accel_history=lat_accel_history,
             ego_state=ego_state, mode_history=np.array(log['mode']),
             u_nmpc=log['u_nmpc'], u_pp=log['u_pp'],
             exit_flag=log['exit_flag'], grad_norm=log['grad_norm'],
             proximity_time=-1.0 if log['proximity_time'] is None
             else log['proximity_time'])
    print(f"Saved: results_nlmpc_pp_{scenario_name}.npz")


if __name__ == '__main__':
    main(headless='--headless' in sys.argv)