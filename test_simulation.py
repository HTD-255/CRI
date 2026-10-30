import numpy as np
import pytest

from simulation import SimConfig, run_simulation, finalize_command
from vehicle_model import DynamicBicycleModel, VehicleParams
from scenario import get_lateral_trajectory, TargetVehicle


@pytest.fixture(scope="module")
def log():
    return run_simulation(SimConfig(max_steps=60))


def test_target_synchronized_with_ego(log):
    cfg = log['config']
    target = TargetVehicle(get_lateral_trajectory(cfg.scenario_name),
                           cfg.target_speed)
    for k in range(len(log['time'])):
        assert np.allclose(log['target_pos'][k], target.position)
        target.step()


def test_applied_commands_replay_ego_state(log):
    params = log['params']
    model = DynamicBicycleModel(params)
    x = log['ego_state'][0].copy()
    for k in range(len(log['time']) - 1):
        x = model.step(x, log['u_applied'][k])
        assert np.allclose(x, log['ego_state'][k + 1], atol=1e-9)


def test_commands_within_limits(log):
    p = log['params']
    u = log['u_applied']
    assert np.all(u >= p.u_min - 1e-12) and np.all(u <= p.u_max + 1e-12)


def test_applied_matches_selected_controller(log):
    for k, m in enumerate(log['mode']):
        if m == 'NMPC':
            assert np.allclose(log['u_applied'][k], log['u_nmpc'][k])
        elif m == 'PP_FALLBACK':
            assert np.allclose(log['u_applied'][k], log['u_pp'][k])


def test_finalize_rejects_non_finite():
    p = VehicleParams()
    with pytest.raises(ValueError):
        finalize_command(np.array([np.nan, 0.0]), p, None, 0.0, 10.0)


def test_finalize_floor_then_clip():
    p = VehicleParams()
    u = finalize_command(np.array([0.0, 0.0]), p, 4.0, 5.0, 10.0)
    assert u[0] == 4.0
    u = finalize_command(np.array([100.0, 9.0]), p, None, 20.0, 10.0)
    assert np.allclose(u, p.u_max)
