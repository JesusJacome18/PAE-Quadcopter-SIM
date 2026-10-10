"""
Pruebas de los arreglos de estimacion de la auditoria del 2026-10-09:
compensacion de aceleracion cinematica (B-01 a B-07), validacion de
estimadores (F-02, F-03, F-09) y magnetometro solo para el rumbo.
"""

import math

import numpy as np
import pytest

from quadsim import math3d as m3
from quadsim.dynamics import IQ
from quadsim.estimation import AccelCompensation
from quadsim.estimators import (BIAS_LIMIT, MahonyEstimator, NavEstimator,
                                make_estimator)
from quadsim.objectives import evaluate_once
from quadsim.scenarios import make_tracking_sim
from quadsim.control.position import make_controller
from quadsim.simulator import SimConfig, Simulation
from quadsim.trajectories import make_trajectory
from quadsim.vehicles import G


def tilt_err_deg(sim):
    zt = m3.q_rotate(tuple(sim.x[IQ:IQ + 4]), (0, 0, 1))
    ze = m3.q_rotate(sim.estimator.quaternion, (0, 0, 1))
    return math.degrees(math.acos(max(-1.0, min(1.0, sum(a * b for a, b in zip(zt, ze))))))


def _hover(pos_sensor, est="mahony", T=30.0):
    sim = Simulation(SimConfig(mode="attitude", start_airborne=True, start_alt=1.5,
                               estimator=est, nav_uses_position_sensor=pos_sensor, seed=1))
    sim.run(T)
    return sim


# ------------------------------------------------------------------ B-01
def test_baro_only_hover_keeps_tilt_estimate():
    """Solo barometro: antes 15.2 deg de error de inclinacion a los 30 s
    (compensacion autorreferente); ahora la compensacion no actua."""
    sim = _hover(False)
    assert sim.acc_comp_gain == 0.0
    assert tilt_err_deg(sim) < 2.0


def test_default_hover_not_worse():
    """Con sensor de posicion (por defecto) la compensacion sigue activa y el
    error no empeora (antes 0.44 deg)."""
    sim = _hover(True)
    assert sim.acc_comp_gain == 1.0
    assert tilt_err_deg(sim) < 0.6


def test_compensation_ramps_on_toggle():
    sim = Simulation(SimConfig(mode="attitude", start_airborne=True, start_alt=1.5))
    sim.run(0.2)
    assert sim.acc_comp_gain == 1.0
    sim.set_acc_compensation(False)
    sim.run(0.25)
    assert sim.acc_comp_gain == pytest.approx(0.5, abs=0.01)     # rampa de 0.5 s
    sim.run(0.3)
    assert sim.acc_comp_gain == 0.0
    sim.set_acc_compensation(True)
    sim.run(0.1)
    assert sim.acc_comp_gain == pytest.approx(0.2, abs=0.01)


def test_compensation_off_on_ground_and_until_nav_settles():
    sim = Simulation(SimConfig(mode="attitude"))
    sim.run(0.5)
    assert not sim.armed and sim.acc_comp_gain == 0.0
    sim.set_attitude_setpoint(alt_m=1.5)
    sim.run(0.8)                          # en el aire pero navegador sin asentar
    assert sim.acc_comp_gain == 0.0
    sim.run(2.0)
    assert sim.acc_comp_gain == 1.0


def test_akin_clipped_and_reset_with_nav():
    nav = NavEstimator(0.002)
    nav.reset((0, 0, -1))
    comp = AccelCompensation(500.0, 30.0, a_max=0.8 * G)
    comp.update(nav, True)
    nav.x[0][1] = 5.0                     # salto de velocidad de 5 m/s en 2 ms
    for _ in range(20):
        comp.update(nav, True)
        assert math.sqrt(sum(a * a for a in comp.a_kin_E)) <= 0.8 * G + 1e-9
    nav.reset((0, 0, -1))                 # aterrizaje/desarme: v -> 0
    comp.update(nav, True)
    assert comp.a_kin_E == (0.0, 0.0, 0.0)


def test_nav_innovation_gate():
    nav = NavEstimator(0.002, pos_noise=0.01)
    nav.reset((0, 0, 0))
    nav.settle(100.0, 500.0)
    assert nav.correct(0, 0.005, 0.01)            # normal
    assert nav.correct(0, 0.10, 0.01)             # 10 cm: bajo el piso, se acepta
    nav.reset((0, 0, 0)); nav.settle(100.0, 500.0)
    assert not nav.correct(0, 0.50, 0.01)         # atipico de 50 cm
    assert nav.position[0] == pytest.approx(nav.position[0])
    p_before = nav.position[0]
    for _ in range(nav.gate_max_rejects - 1):
        assert not nav.correct(0, 0.50, 0.01)
    assert nav.position[0] == p_before
    assert nav.correct(0, 0.50, 0.01)             # tras N rechazos se acepta


def test_gate_does_not_reject_biased_prediction_under_emi():
    """Regresion: la compuerta sin piso descartaba medidas buenas cuando el
    rumbo estimado estaba sesgado por EMI y el MPC perdia la figura en 8
    rapida (RMSE ~1.6 m). Con el piso de 0.25 m vuelve a ~0.16 m."""
    import math, numpy as np
    traj = make_trajectory("lemniscate", period=5.0, laps=3)
    sim = make_tracking_sim(traj, make_controller("mpc", "pae_x450"), "mag_emi", seed=1)
    n = int(traj.duration / sim.dt_ctrl); err = []
    for i in range(n):
        sim.step_control()
        if i % 10 == 0:
            err.append(math.dist(traj.sample(sim.t).p, sim.x[0:3]))
    assert float(np.sqrt(np.mean(np.square(err)))) < 0.30


# ------------------------------------------------------------------ B-02
def test_akin_uses_accel_filter_and_is_time_aligned():
    """a_kin pasa por el mismo Butterworth que el acelerometro; el desfase
    entre ambas senales en la figura en 8 rapida bajo de ~66 ms a ~17 ms."""
    c = SimConfig()
    comp = AccelCompensation(c.ctrl_hz, c.ahrs_accel_lpf_hz)
    assert comp.lpf[0].ftype == "butterworth" and comp.lpf[0].fc == c.ahrs_accel_lpf_hz
    from quadsim.filters import Biquad
    traj = make_trajectory("lemniscate", period=5.0, laps=1)
    sim = make_tracking_sim(traj, make_controller("pid", "pae_x450"), "nominal", seed=1)
    lp = [Biquad(500.0, "butterworth", 30.0) for _ in range(3)]
    T, AK, AA = [], [], []

    def cb(s):
        T.append(s.t)
        AK.append(s.a_kin_E)
        if s.last_imu is None:
            AA.append((0.0, 0.0, 0.0))
            return
        af = [lp[i].process(s.last_imu.accel[i]) for i in range(3)]
        aE = m3.q_rotate(s.estimator.quaternion, af)
        AA.append((aE[0], aE[1], aE[2] + G))
    sim.run(traj.duration, callback=cb)
    t, ak, aa = np.array(T), np.array(AK), np.array(AA)
    idx = np.where((t > 3.0) & (t < traj.duration - 1.0))[0]
    errs = [sum(np.mean((aa[idx, a] - ak[idx + l, a]) ** 2) for a in (0, 1))
            for l in range(0, 40)]
    lag_ms = 2.0 * int(np.argmin(errs))
    assert lag_ms < 30.0, lag_ms


# ------------------------------------------------------------------ B-03
def test_shadow_mode_matches_in_loop():
    """Antes Madgwick daba 1.26 deg en sombra contra 0.43 en el lazo."""
    r = {il: evaluate_once("estimator_accuracy", {"in_loop": il},
                           SimConfig(estimator="madgwick"))["est_err_rms_deg"]
         for il in (False, True)}
    assert r[False] == pytest.approx(r[True], rel=0.15), r


# ------------------------------------------------------------------ B-07
def test_settle_respects_position_sensor():
    traj = make_trajectory("hover", alt=1.5)
    a = make_tracking_sim(traj, make_controller("pid", "pae_x450"),
                          nav_uses_position_sensor=True)
    b = make_tracking_sim(traj, make_controller("pid", "pae_x450"),
                          nav_uses_position_sensor=False)
    # sin sensor de posicion la covarianza horizontal NO converge (crece)
    assert b.nav.P[0][0][0] > 100 * a.nav.P[0][0][0]
    assert b.nav.settled and a.nav.settled
    assert math.isinf(b.nav.t_since_xy_fix)


# ------------------------------------------------------- F-02, F-03, F-09
def test_set_gains_rejects_unknown_and_internal_keys():
    e = make_estimator("mahony", 0.002, 0.0)
    with pytest.raises(TypeError, match="kp"):
        e.set_gains(kpp=5.0)
    with pytest.raises(TypeError):
        e.set_gains(q=1.0)
    e.set_gains(kp=2.0)
    assert e.kp == 2.0
    with pytest.raises(TypeError):
        make_estimator("ideal", 0.002, 0.0).set_gains(beta=1.0)


def test_make_estimator_unknown_raises():
    with pytest.raises(KeyError, match="ekf"):
        make_estimator("typo_ekf", 0.002, 0.0)


def test_mahony_integrator_clipped():
    e = MahonyEstimator(0.002, kp=0.0, ki=50.0, use_mag=False)
    e.initialize((0.0, 0.0, -G), None)
    for _ in range(2000):            # gravedad medida a 90 deg: error maximo
        e.update((0.0, 0.0, 0.0), (-G, 0.0, 0.0), None, False)
    assert max(abs(b) for b in e.bias) <= BIAS_LIMIT + 1e-12


# ------------------------------------------- magnetometro solo para rumbo
@pytest.mark.parametrize("est,limit", [("mahony", 3.0), ("madgwick", 2.0)])
def test_mag_emi_does_not_tilt_estimate(est, limit):
    """Con 40 uT de EMI: antes Mahony 9.7 deg y Madgwick 2.9 deg de error de
    inclinacion RMS; con la correccion proyectada al rumbo 1.6 y 1.1 deg."""
    r = evaluate_once("estimator_accuracy",
                      {"in_loop": True, "emi": True, "emi_gain_uT": 40.0},
                      SimConfig(estimator=est))
    assert r["est_err_rms_deg"] < limit, r


def test_mag_correction_is_heading_only():
    """Un campo magnetico con inclinacion equivocada no mueve roll/pitch."""
    for cls_name in ("mahony", "madgwick"):
        e = make_estimator(cls_name, 0.002, 0.0)
        e.initialize((0.0, 0.0, -G), (1.0, 0.0, 0.5))
        q0 = e.quaternion
        for _ in range(500):
            e.update((0.0, 0.0, 0.0), (0.0, 0.0, -G), (0.3, 0.6, 1.5), True)
        r0, p0, _ = m3.q_to_euler(q0)
        r1, p1, _ = m3.q_to_euler(e.quaternion)
        assert abs(r1 - r0) < math.radians(0.05) and abs(p1 - p0) < math.radians(0.05), cls_name
