import numpy as np
import pytest

from vehicle_model import DynamicBicycleModel, VehicleParams


def make(kind):
    p = VehicleParams()
    p.tire_model = kind
    return DynamicBicycleModel(p)


def test_small_slip_matches_linear():
    sat, lin = make('saturated'), make('linear')
    fs = sat.tire_forces(15.0, 0.0, 0.0, 0.002)
    fl = lin.tire_forces(15.0, 0.0, 0.0, 0.002)
    assert np.allclose(fs, fl, rtol=1e-2)


def test_force_never_exceeds_friction_limit():
    m = make('saturated')
    p = m.params
    for ax in (-5.0, 0.0, 8.0):
        Ff, Fr = m.axle_force_limits(ax)
        for delta in np.linspace(-0.45, 0.45, 19):
            Fyf, Fyr = m.tire_forces(20.0, 0.0, 0.0, delta, ax)
            assert abs(Fyf) <= Ff + 1e-9 and abs(Fyr) <= Fr + 1e-9


def test_lateral_accel_bounded_by_mu_g():
    m = make('saturated')
    p = m.params
    x = np.array([0, 0, 0, 25.0, 0, 0])
    for delta in np.linspace(0.0, 0.45, 10):
        assert m.lateral_accel(x, [0.0, delta]) <= p.mu * p.g * 1.0001


def test_linear_exceeds_friction_limit():
    lin = make('linear')
    Fyf, _ = lin.tire_forces(25.0, 0.0, 0.0, 0.45, 0.0)
    Ff_max, _ = lin.axle_force_limits(0.0)
    assert Fyf > Ff_max


def test_friction_circle_reduces_lateral_with_accel():
    m = make('saturated')
    assert m.axle_force_limits(6.0)[0] < m.axle_force_limits(0.0)[0]


def test_longitudinal_accel_limited_by_grip():
    m = make('saturated')
    x = np.array([0, 0, 0, 10.0, 0, 0])
    dx = m.continuous_dynamics(x, [8.0, 0.0])
    assert dx[3] <= m.params.long_grip_frac * m.params.mu * m.params.g + 1e-9


def test_utilization_flags_saturation():
    m = make('saturated')
    x = np.array([0, 0, 0, 25.0, 0, 0])
    assert m.tire_utilization(x, [0.0, 0.01]) < 1.0
    assert m.tire_utilization(x, [0.0, 0.45]) > 1.0
