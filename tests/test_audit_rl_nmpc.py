"""
Pruebas de los arreglos de la auditoria 2026-10-09 en el NMPC (P-04) y en la
capa de RL (C-09, C-11, C-12, C-13, C-14).
"""

import inspect
import json
import os

import numpy as np
import pytest

from quadsim.control.position import UNAVAILABLE, CONTROLLERS
from quadsim.control.position.nmpc import NonlinearMPC
from quadsim.rl.common import MODELS_DIR, OBS_DIM_PURE, OBS_DIM_RES
from quadsim.rl.policy import MLPPolicy, resolve_model_path
from quadsim.simulator import NavState
from quadsim.trajectories import make_trajectory
from quadsim.vehicles import get_vehicle

VEH = get_vehicle("pae_x450")


# ------------------------------------------------------------------ P-04
def _nmpc_calls(c, n=30):
    traj = make_trajectory("step_waypoints", size=2.0, hold=3.0)
    out = []
    for k in range(n):
        t = 3.0 + 0.01 * k
        nav = NavState(t, (0.0, 0.0, -1.5), (0, 0, 0), (0, 0, 0), (0, 0, 0), VEH.mass)
        cmd = c.update(t, nav, traj)
        out.append((cmd.thrust,) + tuple(cmd.att))
    return np.array(out)


def test_ilqr_lm_raises_mu_when_quu_not_pd():
    c = NonlinearMPC(VEH)
    orig = c._backward
    fails = {"n": 0}

    def flaky(A, B, X, U, Xr, Ur, mu):
        if mu < 1e-3:                    # simula Quu indefinida con mu chico
            fails["n"] += 1
            return None
        return orig(A, B, X, U, Xr, Ur, mu)
    c._backward = flaky
    u = _nmpc_calls(c, 5)
    assert np.all(np.isfinite(u))
    assert c.lm_failures > 0 and c._mu >= 1e-4


def test_ilqr_keeps_previous_solution_when_all_fail():
    c = NonlinearMPC(VEH)
    _nmpc_calls(c, 3)
    U_prev = c.U.copy()
    c._backward = lambda *a: None
    u = _nmpc_calls(c, 1)
    assert np.all(np.isfinite(u))
    assert np.all(c.U >= c.u_lo - 1e-9) and np.all(c.U <= c.u_hi + 1e-9)
    assert c.U.shape == U_prev.shape


def test_ilqr_default_path_no_failures():
    c = NonlinearMPC(VEH)
    _nmpc_calls(c, 30)
    assert c.lm_failures == 0


# ------------------------------------------------------------------ C-13
def test_model_path_ignores_cwd(tmp_path, monkeypatch):
    fake = tmp_path / "rl_pure.npz"
    fake.write_bytes(b"no es un modelo")
    monkeypatch.chdir(tmp_path)
    assert resolve_model_path("rl_pure.npz") == MODELS_DIR / "rl_pure.npz"
    assert resolve_model_path(str(fake)) == fake           # absoluta: explicita
    with pytest.raises(FileNotFoundError):
        resolve_model_path("no_existe.npz")


def test_policy_obs_dim_and_mode_validated():
    assert MLPPolicy.load("rl_pure.npz", obs_dim=OBS_DIM_PURE, mode="pure").obs_dim == OBS_DIM_PURE
    with pytest.raises(ValueError):
        MLPPolicy.load("rl_pure.npz", obs_dim=OBS_DIM_RES)
    with pytest.raises(ValueError):
        MLPPolicy.load("rl_pure.npz", mode="residual")


def test_registry_reports_unavailable():
    assert isinstance(UNAVAILABLE, dict)
    for k in ("pid", "mpc", "nmpc", "rl", "mpc_rl"):
        assert (k in CONTROLLERS) != (k in UNAVAILABLE)


def test_benchmark_fails_on_missing_controller(monkeypatch):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "experiments"))
    import benchmark_controllers as bc
    with pytest.raises(SystemExit, match="no_existe"):
        bc.main(["--controllers", "pid", "no_existe", "--no-plots", "--out", "/nonexistent"])


# ------------------------------------------------------- C-09, C-11, C-12
def test_policy_surrogate_from_metadata():
    from quadsim.rl.validation import _policy_surrogate
    pol = MLPPolicy.load("rl_pure.npz")
    sp = _policy_surrogate(pol)
    assert sp.tau_xy == pytest.approx(pol.meta["surrogate"]["tau_xy"])
    assert sp.delay == pytest.approx(pol.meta["surrogate"]["delay"])


def test_validation_uses_sample_std_and_real_estimator():
    from quadsim.rl import validation
    src = inspect.getsource(validation.run_validation)
    assert "std(ddof=1)" in src and "estimador Madgwick" not in src
    m, _ = validation.run_full("pid", "circle")
    assert m["estimator"] == "mahony"


# ------------------------------------------------------------------ C-14
def test_identify_writes_versioned_file(tmp_path):
    from quadsim.rl import surrogate
    canon = surrogate.ID_FILE.read_text(encoding="utf-8")
    p, b, data, path = surrogate.identify(save=True, duration=4.0,
                                          out_file=tmp_path / "surrogate_id_v9.json")
    assert path.exists() and json.loads(path.read_text())["tau_xy"] == p.tau_xy
    assert data["estimator"] == "mahony" and "mahony" in p.source
    assert surrogate.ID_FILE.read_text(encoding="utf-8") == canon
    nxt = surrogate.next_versioned_file()
    assert nxt != surrogate.ID_FILE and nxt.name.startswith("surrogate_id_v")
