"""
Tests de los controladores de posicion (quadsim.control.position), de la
identificacion y de las metricas.

  * registro y esquema PARAMS (contrato con la GUI y con los controladores de RL);
  * discretizacion ZOH exacta del modelo del MPC (vs scipy expm);
  * lq_gain() reproduce el QP sin restricciones (contrato con el RL);
  * el MPC respeta la caja de entrada (QP aislado y en la simulacion);
  * presupuesto de tiempo de computo (lazo externo de 10 ms);
  * cada controlador sigue un circulo en la simulacion COMPLETA con error
    acotado;
  * el modelo del NMPC conserva el equilibrio de hover y su iLQR respeta
    las cotas;
  * fit_fopdt recupera parametros sinteticos; metricas sobre senales
    conocidas.
"""

import math
import time

import numpy as np
import pytest

from quadsim.control.position import CONTROLLERS, make_controller
from quadsim.control.position.base import PositionController
from quadsim.control.position.mpc import LinearMPC, lag_model_zoh
from quadsim.control.position.nmpc import NonlinearMPC, flat_inputs, r3_zyx
from quadsim.identification import DEFAULT_LAGS, fit_fopdt, get_lags
from quadsim.metrics import step_metrics, tracking_metrics
from quadsim.scenarios import make_tracking_sim
from quadsim.simulator import NavState
from quadsim.trajectories import make_trajectory
from quadsim.vehicles import G, get_vehicle

VEH = get_vehicle("pae_x450")
CORE = ["pid", "mpc", "nmpc"]


# ----------------------------------------------------------------- registro
@pytest.mark.parametrize("key", CORE)
def test_registry_and_schema(key):
    cls = CONTROLLERS[key]
    assert issubclass(cls, PositionController)
    assert cls.name == key and cls.label
    for p in cls.PARAMS:
        assert len(p) == 7, p
        assert p[3] <= p[2] <= p[4], p
    c = cls(VEH)
    assert set(c.get_params()) == {p[0] for p in cls.PARAMS}
    with pytest.raises(TypeError):
        cls(VEH, no_existe=1)
    c2 = make_controller(key, "pae_x450")
    assert type(c2) is cls


def test_registry_survives_missing_learning_modules():
    # rl / mpc_rl pueden faltar; si estan, deben ser PositionController
    for k in ("rl", "mpc_rl"):
        if k in CONTROLLERS:
            assert issubclass(CONTROLLERS[k], PositionController)


# ----------------------------------------------------------------- MPC lineal
def test_zoh_matches_expm():
    from scipy.linalg import expm
    for tau, dt in ((0.119, 0.05), (0.0195, 0.05), (0.3, 0.01)):
        Ac = np.array([[0, 1, 0], [0, 0, 1], [0, 0, -1 / tau]])
        Bc = np.array([0, 0, 1 / tau])
        M = np.zeros((4, 4))
        M[:3, :3], M[:3, 3] = Ac, Bc
        E = expm(M * dt)
        A, B = lag_model_zoh(tau, dt)
        assert np.allclose(A, E[:3, :3], atol=1e-12)
        assert np.allclose(B, E[:3, 3], atol=1e-12)


@pytest.mark.parametrize("kw", [{}, {"r_du": 0.01, "u_dev": 0.0}, {"q_v": 0.3}])
def test_lq_gain_reproduces_unconstrained_qp(kw):
    mpc = LinearMPC(VEH, **kw)
    traj = make_trajectory("lemniscate")
    rng = np.random.default_rng(3)
    g = mpc.lq_gain()
    for _ in range(5):
        t = rng.uniform(0, traj.duration)
        dP, Vr, Ur, S = mpc.reference_preview(traj, t)
        for i, ax in enumerate("xyz"):
            e = np.array([rng.normal(0, 0.1), rng.normal(0, 0.2), rng.normal(0, 0.3)])
            uprev = rng.normal(0, 0.5)
            p = S["p"][0, i] + e[0]
            v = S["v"][0, i] + e[1]
            x0 = np.array([p - S["p"][0, i], v, e[2]])
            q = mpc.qp[i]
            f = q.f(x0, dP[1:, i], Vr[1:, i], Ur[:, i], uprev)
            u_qp = (-q.Hinv @ f)[0]
            G_ = g[ax]
            u_lq = float((G_["K"] @ e)[0] + G_["Gp"] @ dP[:, i] + G_["Gv"] @ Vr[:, i]
                         + G_["Ga"] @ Ur[:, i] + G_["Gu"] * uprev)
            assert u_lq == pytest.approx(u_qp, abs=1e-9)
        assert abs(g["x"]["sum_check"]) < 1e-9


def test_mpc_qp_respects_box_and_is_near_optimal():
    from scipy.optimize import minimize
    mpc = LinearMPC(VEH)
    mpc.d_hat = np.zeros(3)
    mpc.dt_last = 0.01
    rng = np.random.default_rng(0)
    for _ in range(20):
        F = np.zeros((3, mpc.N))
        for i in range(3):
            x0 = rng.uniform(-3, 3, 3)
            F[i] = mpc.qp[i].f(x0, np.zeros(mpc.N), np.zeros(mpc.N),
                               np.zeros(mpc.N), 0.0)
        U, it, active = mpc._solve(F)
        assert np.all(U >= mpc.lo[:, None] - 1e-9) and np.all(U <= mpc.hi[:, None] + 1e-9)
        for i in range(3):
            H, f = mpc.qp[i].H, F[i]
            J = lambda u: 0.5 * u @ H @ u + f @ u
            r = minimize(J, np.zeros(mpc.N), jac=lambda u: H @ u + f,
                         bounds=[(mpc.lo[i], mpc.hi[i])] * mpc.N, method="L-BFGS-B",
                         options={"ftol": 1e-15, "gtol": 1e-12, "maxiter": 5000})
            assert (J(U[i]) - r.fun) <= 0.01 * abs(J(np.zeros(mpc.N)) - r.fun) + 1e-9


def _run(ctrl_key, traj, scenario="nominal", **kw):
    ctrl = make_controller(ctrl_key, VEH, **kw)
    sim = make_tracking_sim(traj, ctrl, scenario)
    ms, cmds = [], []
    for _ in range(int(round(traj.duration / sim.dt_ctrl))):
        sim.step_control()
        if sim.k % sim.outer_every == 1:
            cmds.append(sim.pos_cmd)
            if hasattr(ctrl, "stats"):
                ms.append(ctrl.stats["solve_ms"])
    return sim, ctrl, sim.log_arrays(), np.array(ms), cmds


def test_mpc_respects_constraints_in_closed_loop():
    traj = make_trajectory("step_waypoints", size=2.0, dz=1.0, hold=3.0)
    sim, ctrl, L, ms, cmds = _run("mpc", traj)
    A = np.array([c.a_des for c in cmds])
    assert np.all(A >= ctrl.lo - 1e-9) and np.all(A <= ctrl.hi + 1e-9)
    # los escalones de 2 m SI saturan (si no, el test no probaria nada)
    assert np.any(np.isclose(np.abs(A[:, 0]), ctrl.hi[0], atol=1e-6))


# ----------------------------------------------------------------- NMPC
def test_nmpc_model_hover_equilibrium_and_small_angle():
    c = NonlinearMPC(VEH, drag=0.0)
    m = VEH.mass
    X = np.array([[0, 0, -1.5, 0, 0, 0, 0, 0, m * G]])
    U = np.array([[m * G, 0.0, 0.0]])
    X1 = c.step(X, U, np.zeros(1), np.zeros(3))
    assert np.allclose(X1, X, atol=1e-12)
    # pitch pequeno y estacionario -> a_x = -g tan(theta) (aprox. -g theta)
    th = math.radians(2.0)
    X = np.array([[0, 0, -1.5, 0, 0, 0, 0, th, m * G / math.cos(th)]])
    U = np.array([[m * G / math.cos(th), 0.0, th]])
    X1 = c.step(X, U, np.zeros(1), np.zeros(3))
    ax = X1[0, 3] / c.dt
    assert ax == pytest.approx(-G * math.tan(th), rel=1e-6)


def test_flat_inputs_roundtrip():
    rng = np.random.default_rng(1)
    acc = rng.uniform(-3, 3, (50, 3))
    psi = rng.uniform(-math.pi, math.pi, 50)
    T, phi, th = flat_inputs(acc, psi, VEH.mass)
    rx, ry, rz = r3_zyx(phi, th, psi)
    a = np.stack([-T * rx, -T * ry, VEH.mass * G - T * rz], 1) / VEH.mass
    assert np.allclose(a, acc, atol=1e-9)


def test_nmpc_respects_bounds():
    c = NonlinearMPC(VEH)
    traj = make_trajectory("step_waypoints", size=2.0, hold=3.0)
    for k in range(60):
        t = 3.0 + 0.01 * k
        nav = NavState(t, (0.0, 0.0, -1.5), (0, 0, 0), (0, 0, 0), (0, 0, 0), VEH.mass)
        cmd = c.update(t, nav, traj)
        u = np.array([cmd.thrust, cmd.att[0], cmd.att[1]])
        assert np.all(u >= c.u_lo - 1e-9) and np.all(u <= c.u_hi + 1e-9)
    assert np.all(c.U >= c.u_lo - 1e-9) and np.all(c.U <= c.u_hi + 1e-9)
    assert np.any(np.isclose(np.abs(c.U[:, 2]), c.u_hi[2]))   # satura en pitch


# ----------------------------------------------------------------- lazo cerrado
@pytest.mark.parametrize("key", CORE)
def test_tracks_circle_full_sim(key):
    traj = make_trajectory("circle", radius=1.5, period=8.0, laps=1, t_ramp=2.0)
    sim, ctrl, L, ms, cmds = _run(key, traj)
    m = tracking_metrics(L)
    assert m["rmse_3d"] < 0.06, m
    assert m["max_err_3d"] < 0.15, m


@pytest.mark.perf
@pytest.mark.parametrize("key", [k for k in CORE if k != "pid"])
def test_solve_time_budget_circle(key):
    # presupuesto: el lazo externo corre cada 10 ms. Depende del hardware
    # (se omite salvo con PERF_TESTS=1); se exige el percentil 50 y se
    # informa el p95 y el entorno.
    from conftest import environment_note
    traj = make_trajectory("circle", radius=1.5, period=8.0, laps=1, t_ramp=2.0)
    sim, ctrl, L, ms, cmds = _run(key, traj)
    msg = (f"{key}: mediana {np.median(ms):.2f} ms, p95 {np.percentile(ms, 95):.2f} ms "
           f"({environment_note()})")
    assert np.median(ms) < 10.0, msg


@pytest.mark.perf
def test_mpc_solve_time_budget():
    from conftest import environment_note
    traj = make_trajectory("lemniscate", period=5.0, laps=1)
    sim, ctrl, L, ms, cmds = _run("mpc", traj)
    assert np.mean(ms) < 3.0 and np.percentile(ms, 99) < 10.0, \
        (np.mean(ms), ms.max(), environment_note())


# ----------------------------------------------------------------- identificacion
def test_fit_fopdt_recovers_synthetic():
    t = np.arange(0, 0.6, 0.002)
    K, tau, th, u = 0.97, 0.07, 0.03, 1.5
    y = K * u * (1 - np.exp(-np.clip(t - th, 0, None) / tau))
    y += np.random.default_rng(0).normal(0, 0.005, len(t))
    r = fit_fopdt(t, y, u)
    assert r["tau"] == pytest.approx(tau, rel=0.05)
    assert r["theta"] == pytest.approx(th, abs=0.002)
    assert r["K"] == pytest.approx(K, rel=0.01)


@pytest.mark.perf
def test_get_lags_fast():
    t0 = time.perf_counter()
    get_lags(VEH)
    assert time.perf_counter() - t0 < 0.01


def test_get_lags_consistent():
    lg = get_lags(VEH)
    assert lg == DEFAULT_LAGS["pae_x450"]
    assert 0.05 < lg["tau_a_xy"] < 0.3 and 0.005 < lg["tau_a_z"] < 0.1
    assert get_lags("pae_x450+payload") == lg


# ----------------------------------------------------------------- metricas
def test_step_metrics_second_order():
    # 2o orden subamortiguado: zeta = 0.5, wn = 4 -> Mp = 16.3 %
    zeta, wn = 0.5, 4.0
    t = np.arange(0, 10, 0.001)
    wd = wn * math.sqrt(1 - zeta ** 2)
    y = 1 - np.exp(-zeta * wn * t) * (np.cos(wd * t) + zeta / math.sqrt(1 - zeta ** 2) * np.sin(wd * t))
    r = step_metrics(t, y, 0.0, 0.0, 1.0)
    assert r["overshoot_pct"] == pytest.approx(100 * math.exp(-zeta * math.pi / math.sqrt(1 - zeta ** 2)), rel=1e-3)
    assert r["ss_error"] == pytest.approx(0.0, abs=1e-4)
    assert 0.0 < r["rise_time"] < 0.6
    assert r["settling_time"] == pytest.approx(4.0 / (zeta * wn), rel=0.25)


def test_tracking_metrics_known_error():
    t = np.linspace(0, 10, 1001)
    L = {"t": t}
    for i in range(3):
        L[f"ref_{i}"] = np.zeros_like(t)
        L[f"pos_{i}"] = np.full_like(t, 0.1 if i == 0 else 0.0)
    m = tracking_metrics(L)
    assert m["rmse_x"] == pytest.approx(0.1) and m["rmse_3d"] == pytest.approx(0.1)
    assert m["iae"] == pytest.approx(1.0) and m["itae"] == pytest.approx(5.0)
