"""
Tests de quadsim.trajectories:
  * derivadas analiticas (v, a, j) == diferencias finitas centradas, en toda
    la trayectoria (incluidas las rampas y los extremos);
  * arranque y llegada en reposo de las curvas periodicas, continuidad en
    los empalmes de la rampa;
  * minimum snap: pasa por los waypoints, derivadas 1..6 continuas en los
    waypoints interiores y su costo es el del QP de Mellinger-Kumar (KKT);
  * contrato del registro (PARAMS, kwargs, duration).
"""

import math

import numpy as np
import pytest

from quadsim.trajectories import (TRAJECTORIES, Circle, Lemniscate, MinimumSnap,
                                  RefPoint, _poly_deriv_row, make_trajectory,
                                  minimum_snap_coefficients)

ALL = sorted(TRAJECTORIES)


@pytest.mark.parametrize("name", ALL)
def test_schema_and_defaults(name):
    cls = TRAJECTORIES[name]
    assert cls.name == name
    assert isinstance(cls.label, str) and cls.label
    for p in cls.PARAMS:
        assert len(p) == 7, p
        key, label, default, lo, hi, step, unit = p
        assert lo <= default <= hi, p
        assert step > 0
    tr = cls()
    assert tr.duration > 0
    assert isinstance(tr.sample(0.0), RefPoint)
    with pytest.raises(TypeError):
        cls(no_existe=1.0)


@pytest.mark.parametrize("name", ALL)
def test_derivatives_match_finite_differences(name):
    tr = make_trajectory(name)
    t = np.linspace(-0.3, tr.duration + 0.3, 3001)
    h = 1e-5
    S = tr.sample_many(t)
    Sp, Sm = tr.sample_many(t + h), tr.sample_many(t - h)
    for a, b in (("p", "v"), ("v", "a"), ("a", "j")):
        fd = (Sp[a] - Sm[a]) / (2 * h)
        scale = max(1.0, float(np.max(np.abs(S[b]))))
        # escalones (StepWaypoints) no tienen derivada en el salto: se
        # excluyen los puntos a menos de h de un cambio de consigna
        ok = np.ones(len(t), bool)
        if name == "step_waypoints":
            ok = np.max(np.abs(Sp["p"] - Sm["p"]), 1) < 1e-12
        err = np.max(np.abs(fd[ok] - S[b][ok]))
        assert err < 1e-5 * scale, (name, a, b, err)


@pytest.mark.parametrize("name", ["circle", "lemniscate", "helix"])
def test_periodic_start_and_end_at_rest(name):
    tr = make_trajectory(name)
    for t in (0.0, tr.duration):
        r = tr.sample(t)
        assert np.allclose(r.v, 0, atol=1e-12)
        assert np.allclose(r.a, 0, atol=1e-12)
        assert np.allclose(r.j, 0, atol=1e-12)
    # vuelve al punto de partida en XY tras `laps` vueltas
    p0, pe = np.array(tr.sample(0).p), np.array(tr.sample(tr.duration).p)
    assert np.allclose(p0[:2], pe[:2], atol=1e-9)
    # continuidad de p, v, a, j en los empalmes de la rampa
    for tk in (tr.Tr, tr.Tr + tr.Tc):
        a, b = tr.sample_many([tk - 1e-9]), tr.sample_many([tk + 1e-9])
        for key in ("p", "v", "a", "j"):
            assert np.allclose(a[key], b[key], atol=1e-6), (name, tk, key)
    # regimen: la velocidad de crucero es la nominal
    tm = tr.Tr + 0.5 * tr.Tc
    if name == "circle":
        sp = np.linalg.norm(tr.sample(tm).v[:2])
        assert abs(sp - tr.params["radius"] * tr.omega) < 1e-9


def test_lemniscate_geometry():
    tr = Lemniscate(A=2.0, B=1.0, period=10.0, laps=1, t_ramp=2.0)
    S = tr.sample_many(np.linspace(0, tr.duration, 20001))
    x, y = S["p"][:, 0], S["p"][:, 1]
    assert abs(x.max() - 2.0) < 1e-3 and abs(x.min() + 2.0) < 1e-3
    assert abs(y.max() - 0.5) < 1e-3 and abs(y.min() + 0.5) < 1e-3
    assert np.allclose(S["p"][:, 2], -1.5)


def _msnap_from_coef(tr, order):
    """Derivada `order` en ambos lados de cada waypoint interior."""
    out = []
    for i in range(len(tr.seg_times) - 1):
        Ti, Tn = tr.seg_times[i], tr.seg_times[i + 1]
        left = _poly_deriv_row(8, order, 1.0) @ tr.coef[i] / Ti ** order
        right = _poly_deriv_row(8, order, 0.0) @ tr.coef[i + 1] / Tn ** order
        out.append((left, right))
    return out


def test_minimum_snap_waypoints_and_continuity():
    tr = MinimumSnap()
    for k, tk in enumerate(tr.t_knots):
        assert np.allclose(tr.sample(tk).p, tr.waypoints[k], atol=1e-9)
    for order in range(1, 7):
        for left, right in _msnap_from_coef(tr, order):
            scale = max(1.0, np.max(np.abs(left)))
            assert np.allclose(left, right, atol=1e-7 * scale), order
    r0, r1 = tr.sample(0.0), tr.sample(tr.duration)
    for r in (r0, r1):
        assert np.allclose(r.v, 0, atol=1e-9) and np.allclose(r.a, 0, atol=1e-9)
        assert np.allclose(r.j, 0, atol=1e-9)


def test_minimum_snap_is_optimal_vs_kkt_qp():
    """
    Mellinger-Kumar: min int ||snap||^2 sujeto a waypoints, reposo en los
    extremos (v=a=j=0) y continuidad de derivadas 0..3 (lo minimo para
    vuelo suave). Se resuelve el QP por KKT y su costo debe coincidir con
    el de nuestra solucion en forma cerrada (que impone C^6, condicion de
    optimalidad, no restriccion).
    """
    W = np.array([[0, 0, -1.5], [1.0, 0.5, -1.8], [2.0, -0.5, -1.6], [3.0, 0.0, -1.5]])
    T = np.array([1.5, 2.0, 1.2])
    M, nc = len(T), 8
    xg, wg = np.polynomial.legendre.leggauss(10)
    tau = 0.5 * (xg + 1)
    Q = np.zeros((M * nc, M * nc))
    for i in range(M):
        R = np.array([_poly_deriv_row(nc, 4, tt) for tt in tau]) / T[i] ** 4
        Q[i * nc:(i + 1) * nc, i * nc:(i + 1) * nc] = R.T @ (wg[:, None] * 0.5 * T[i] * R)
    rows, rhs = [], []

    def row(i, order, tt, Ti):
        r = np.zeros(M * nc)
        r[i * nc:(i + 1) * nc] = _poly_deriv_row(nc, order, tt) / Ti ** order
        return r
    for i in range(M):
        rows += [row(i, 0, 0.0, T[i]), row(i, 0, 1.0, T[i])]
        rhs += [W[i], W[i + 1]]
    for o in (1, 2, 3):
        rows += [row(0, o, 0.0, T[0]), row(M - 1, o, 1.0, T[M - 1])]
        rhs += [np.zeros(3), np.zeros(3)]
    for i in range(M - 1):
        for o in (1, 2, 3):
            rows.append(row(i, o, 1.0, T[i]) - row(i + 1, o, 0.0, T[i + 1]))
            rhs.append(np.zeros(3))
    A = np.array(rows)
    b = np.array(rhs)
    nA = A.shape[0]
    K = np.block([[2 * Q, A.T], [A, np.zeros((nA, nA))]])
    sol = np.linalg.lstsq(K, np.vstack([np.zeros((M * nc, 3)), b]), rcond=None)[0]
    c_qp = sol[:M * nc]
    J_qp = float(np.sum(c_qp * (Q @ c_qp)))
    tr = MinimumSnap(waypoints=W, seg_times=T)
    J_cf = tr.snap_cost()
    assert J_cf == pytest.approx(J_qp, rel=1e-6)
    c_cf = minimum_snap_coefficients(W, T).reshape(M * nc, 3)
    assert np.allclose(c_cf, c_qp, atol=1e-6 * np.max(np.abs(c_cf)))


def test_quintic_endpoints():
    tr = make_trajectory("quintic")
    assert np.allclose(tr.sample(0).p, tr.p0)
    assert np.allclose(tr.sample(tr.duration).p, tr.p1)
    mid = tr.th + 0.5 * tr.T
    # velocidad pico del minimo jerk = 1.875 |dp| / T
    vp = np.linalg.norm(tr.sample(mid).v)
    assert vp == pytest.approx(1.875 * np.linalg.norm(tr.p1 - tr.p0) / tr.T, rel=1e-9)


def test_step_waypoints_steps_listing():
    tr = make_trajectory("step_waypoints", size=1.0, dz=0.5, hold=3.0)
    steps = tr.steps()
    assert len(steps) == len(tr.points) - 1
    for ts, a, b, te in steps:
        assert np.allclose(tr.sample(ts + 1e-6).p, b)
        assert np.allclose(tr.sample(ts - 1e-6).p, a)


def test_ned_altitude_convention():
    for name in ALL:
        tr = make_trajectory(name)
        assert tr.start_point()[2] < 0, name    # altura positiva -> z negativo
