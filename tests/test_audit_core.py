"""
Pruebas de los arreglos del nucleo de la auditoria del 2026-10-09: base de
tiempo (A-04), lineas de retardo (A-07, A-08, A-12), registro (F-04 y
anillo), semilla global (F-10), referencia por instante (F-01), validacion
de vehiculos, rafaga de duracion cero y reset conservando ganancias.
"""

import math
import random

import numpy as np
import pytest

from quadsim.disturbances import DisturbanceConfig, DisturbanceManager, WindConfig
from quadsim.dynamics import IR
from quadsim.sensors import SensorConfig, _Delay
from quadsim.simulator import SimConfig, Simulation
from quadsim.trajectories import make_trajectory
from quadsim.vehicles import get_vehicle


class _FreeFall(Simulation):
    def _outer_loop(self):
        self.armed = False
        self.thrust_cmd = 0.0


def _fall(ctrl_hz):
    s = _FreeFall(SimConfig(mode="attitude", start_airborne=True, start_alt=100.0,
                            ctrl_hz=ctrl_hz))
    s.x[IR:IR + 4] = [0.0] * 4
    s.run(1.0)
    return 100.0 + s.x[2], s


# ------------------------------------------------------------------ A-04
@pytest.mark.parametrize("hz", [400.0, 2000.0, 333.0])
def test_free_fall_independent_of_ctrl_rate(hz):
    """Antes: 400 Hz -> 3.017 m (20 % lenta), 2000 Hz -> 16.0 m (el doble)."""
    ref, _ = _fall(500.0)
    d, s = _fall(hz)
    assert ref == pytest.approx(4.617, abs=0.01)
    assert d == pytest.approx(ref, rel=1e-3)
    assert s.dt_phys * s.n_sub == pytest.approx(s.dt_ctrl, rel=1e-12)


def test_invalid_rates_raise():
    with pytest.raises(ValueError):
        Simulation(SimConfig(ctrl_hz=0.0))


# ------------------------------------------------------- A-07, A-08, A-12
def test_delay_zero_and_fractional():
    d0 = _Delay(0.0, 0.002, 0.0)
    assert [d0.step(v) for v in (1.0, 2.0, 3.0)] == [1.0, 2.0, 3.0]
    d1 = _Delay(0.003, 0.002, 0.0)          # 1.5 muestras
    out = [d1.step(float(k)) for k in range(6)]
    assert out[3:] == pytest.approx([1.5, 2.5, 3.5])
    d2 = _Delay(0.004, 0.002, 0.0)          # entero: exacto
    assert [d2.step(float(k)) for k in range(5)] == [0.0, 0.0, 0.0, 1.0, 2.0]


def test_imu_delay_primed_monotonic():
    """Sin cebar, con retardo de 2+ muestras el tiempo retrocedia."""
    d = _Delay(0.006, 0.002, None)
    out = [d.step(k) for k in range(8)]
    assert out == [0, 0, 0, 0, 1, 2, 3, 4]


def test_pos_link_delay_adds():
    s = Simulation(SimConfig(sensors=SensorConfig(pos_delay_s=0.01, pos_link_delay_s=0.02)))
    assert s.sensors._pos_delay.d == pytest.approx(15.0)


# ------------------------------------------------------- F-04 y registro
def test_log_columns_nan_padded_when_trajectory_starts_late():
    from quadsim.control.position import make_controller
    sim = Simulation(SimConfig(mode="trajectory", start_airborne=True, start_alt=1.5))
    sim.run(0.2)
    sim.set_position_controller(make_controller("pid", "pae_x450"))
    sim.set_trajectory(make_trajectory("hover", alt=1.5))
    L = sim.run(0.2)
    n = len(L["t"])
    assert all(len(v) == n for v in L.values())
    assert np.isnan(L["ref_0"][0]) and np.isfinite(L["ref_0"][-1])


def test_log_ring_and_switch():
    s = Simulation(SimConfig(start_airborne=True, start_alt=1.5, log_max_samples=50))
    L = s.run(3.0)
    assert len(L["t"]) == 50 and len(s.log["t"]) <= 100
    assert L["t"][-1] == pytest.approx(s.t, abs=0.011)
    s2 = Simulation(SimConfig(start_airborne=True, start_alt=1.5, log_enabled=False))
    s2.run(0.5)
    assert s2.log == {} and s2.log_arrays() == {}


# ------------------------------------------------------------------ F-10
def test_reset_does_not_seed_global_random():
    random.seed(123)
    a = random.random()
    random.seed(123)
    Simulation(SimConfig(seed=7))
    assert random.random() == a


# ------------------------------------------------------------------ F-01
def test_reference_sampled_once_per_outer_tick():
    from quadsim.scenarios import make_tracking_sim
    from quadsim.control.position import make_controller
    traj = make_trajectory("lemniscate", period=5.0, laps=1)
    sim = make_tracking_sim(traj, make_controller("pid", "pae_x450"), "nominal")
    calls = {"n": 0}
    orig = traj._sample

    def counted(t):
        calls["n"] += 1
        return orig(t)
    traj._sample = counted
    sim.run(1.0)
    assert calls["n"] == 100            # 1 por periodo del lazo externo (antes 4)


@pytest.mark.parametrize("name", ["circle", "lemniscate", "helix"])
def test_scalar_path_matches_vectorized(name):
    tr = make_trajectory(name)
    ts = np.linspace(-1.0, tr.duration + 1.0, 301)
    M = tr.sample_many(ts)
    for i, t in enumerate(ts):
        r = tr._sample(float(t))
        for key in ("p", "v", "a", "j"):
            assert np.allclose(getattr(r, key), M[key][i], atol=1e-12)


# ------------------------------------------------- vehiculos y viento
def test_unknown_vehicle_raises():
    with pytest.raises(KeyError, match="pae_x450"):
        get_vehicle("no_existe")
    with pytest.raises(KeyError):
        Simulation(SimConfig(vehicle="no_existe"))


def test_zero_duration_gust_does_not_divide_by_zero():
    cfg = DisturbanceConfig(wind=WindConfig(gust_amplitude=4.0, gust_start=1.0,
                                            gust_duration=0.0))
    dm = DisturbanceManager(cfg, 1)
    w = dm._wind(1.0, 0.002, 2.0, 0.0)
    assert all(math.isfinite(v) for v in w)


# ------------------------------------------------- reset conservando ganancias
def test_reset_keep_gains():
    s = Simulation(SimConfig())
    s.att_ctrl.set_gains("roll", angle_kp=5.0, rate_kp=0.3, rate_ki=1.0, rate_kd=0.004)
    s.vert_ctrl.kp_pos = 1.7
    s.reset(keep_gains=True)
    g = s.att_ctrl.get_gains()["roll"]
    assert (g["angle_kp"], g["rate_kp"], g["rate_ki"], g["rate_kd"]) == (5.0, 0.3, 1.0, 0.004)
    assert s.vert_ctrl.kp_pos == 1.7
    s.reset()                                   # por defecto: SIMC
    assert s.att_ctrl.get_gains()["roll"]["rate_kp"] == pytest.approx(0.2114)
