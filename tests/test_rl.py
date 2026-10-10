"""
Pruebas del paquete quadsim.rl y de los controladores "rl" y "mpc_rl".

    PYTHONPATH=. pytest -q tests/test_rl.py
"""

from __future__ import annotations

import math
import subprocess
import sys

import numpy as np
import pytest

from quadsim.rl.common import MODELS_DIR, REPO_ROOT, clip_accel, TILT_MAX
from quadsim.vehicles import G


# ---------------------------------------------------------------- inferencia
def test_policy_files_exist_and_load_numpy_only():
    """Los pesos cargan y corren en un interprete SIN torch (como en Windows)."""
    code = (
        "import sys; sys.modules['torch'] = None\n"      # import torch fallaria
        "import numpy as np\n"
        "from quadsim.rl.policy import MLPPolicy\n"
        "for f in ('rl_pure.npz', 'mpc_rl_residual.npz'):\n"
        "    p = MLPPolicy.load(f)\n"
        "    y = p.act(np.zeros(p.obs_dim))\n"
        "    assert y.shape == (3,) and np.all(np.abs(y) <= 1)\n"
        "print('ok')\n")
    r = subprocess.run([sys.executable, "-c", code], cwd=str(REPO_ROOT), capture_output=True,
                       text=True, env={"PYTHONPATH": str(REPO_ROOT), "PATH": ""})
    assert r.returncode == 0, r.stderr
    assert "ok" in r.stdout


def test_policy_deterministic():
    from quadsim.rl.policy import MLPPolicy
    p = MLPPolicy.load("rl_pure.npz")
    o = np.random.default_rng(0).standard_normal((5, p.obs_dim))
    a1, a2 = p.act(o), p.act(o.copy())
    assert np.array_equal(a1, a2)
    # fila a fila == lote
    assert np.allclose(np.stack([p.act(x) for x in o]), a1)


def test_model_path_independent_of_cwd(tmp_path, monkeypatch):
    from quadsim.rl.policy import resolve_model_path
    monkeypatch.chdir(tmp_path)
    assert resolve_model_path("rl_pure.npz") == MODELS_DIR / "rl_pure.npz"
    assert resolve_model_path("rl_pure") == MODELS_DIR / "rl_pure.npz"


def test_metadata_matches_env():
    from quadsim.rl.common import OBS_DIM_PURE, OBS_DIM_RES
    from quadsim.rl.policy import MLPPolicy
    assert MLPPolicy.load("rl_pure.npz").obs_dim == OBS_DIM_PURE
    m = MLPPolicy.load("mpc_rl_residual.npz")
    assert m.obs_dim == OBS_DIM_RES and m.meta["mode"] == "residual"


# ---------------------------------------------------------------- piezas
def test_clip_accel_tilt_cone():
    a = clip_accel(np.array([[20.0, 0, 0], [3.0, 4.0, -3.0], [0, 0, 50.0]]))
    for row in a:
        tilt = math.atan2(math.hypot(row[0], row[1]), G - row[2])
        assert tilt <= TILT_MAX + 1e-9


def test_numpy_mlp_gradients():
    from quadsim.rl.ppo import MLP
    rng = np.random.default_rng(0)
    net = MLP([4, 6, 6, 2], rng, out_gain=0.5, out_tanh=True)
    net.params = [p.astype(np.float64) for p in net.params]
    x = rng.standard_normal((3, 4))
    w = rng.standard_normal((3, 2))
    y, c = net.forward(x.astype(np.float32))
    g = net.backward(w, c)
    P = net.params[0]
    i = (1, 2)
    o = P[i]
    P[i] = o + 1e-3
    fp = float(np.sum(net.forward(x)[0] * w))
    P[i] = o - 1e-3
    fm = float(np.sum(net.forward(x)[0] * w))
    P[i] = o
    assert abs((fp - fm) / 2e-3 - g[0][i]) < 1e-3


def test_refgen_derivatives():
    from quadsim.rl.refgen import RefBatch
    rb = RefBatch(20)
    rng = np.random.default_rng(1)
    rb.randomize(np.arange(20), rng)
    rb.Tj[rb.Tj < 0.05] = 0.5              # sin escalones duros para derivar
    t = rng.uniform(0, 15, 20)
    h = 1e-5
    p, v, a, j = rb.sample(t, jerk=True)
    _, vp, ap, _ = rb.sample(t + h, jerk=True)
    _, vm, am, _ = rb.sample(t - h, jerk=True)
    assert np.abs((vp - vm) / (2 * h) - a).max() < 1e-4
    assert np.abs((ap - am) / (2 * h) - j).max() < 1e-3


def test_mpc_replica_matches_real_mpc_when_unconstrained():
    """La replica vectorizada == LinearMPC real mientras el QP no satura."""
    from quadsim.rl.base_mpc import MPCReplica, make_mpc
    from quadsim.rl.refgen import RefBatch, RefTrajectory
    from quadsim.simulator import NavState
    rng = np.random.default_rng(4)
    rb = RefBatch(1)
    rb.A[0, 0, :2] = [1.0, 0.8]
    rb.w[0, 0, :2] = [0.8, 1.1]
    rb.c[0, 2] = -1.5
    mpc = make_mpc()
    rep = MPCReplica(mpc)
    rep.alloc(1)
    traj = RefTrajectory(rb, 0)
    p = np.array(traj.sample(0).p) + 0.05
    v = np.array(traj.sample(0).v)          # arranca en movimiento: sin saturar
    a = np.zeros(3)
    for k in range(300):
        t = k * 0.01
        u_real = np.array(mpc.update(t, NavState(t, tuple(p), tuple(v), (0, 0, 0),
                                                 (0, 0, 0), 1.4), traj).a_des)
        u_rep = rep.compute(np.array([t]), p[None], v[None], rb)[0]
        rep.set_applied(u_rep)
        assert not mpc.stats["active"]
        assert np.abs(u_real - u_rep).max() < 1e-9
        a += (1 - math.exp(-0.01 / 0.09)) * (u_real - a)
        v += (a + rng.normal(0, 0.1, 3)) * 0.01
        p += v * 0.01


def test_env_runs_and_is_seeded():
    from quadsim.rl.env import QuadTrackEnv
    outs = []
    for _ in range(2):
        env = QuadTrackEnv(8, seed=3)
        o = env.reset()
        for _ in range(50):
            o, r, d, tr, info = env.step(np.zeros((8, 3)))
        outs.append(o)
        assert np.all(np.isfinite(o)) and o.shape == (8, env.obs_dim)
    assert np.array_equal(outs[0], outs[1])


# ---------------------------------------------------------------- en el 6-DOF
@pytest.mark.parametrize("key", ["rl", "mpc_rl"])
def test_controller_in_full_simulation(key):
    from quadsim.control.position import CONTROLLERS, make_controller
    from quadsim.scenarios import make_tracking_sim
    from quadsim.trajectories import make_trajectory
    assert key in CONTROLLERS
    cls = CONTROLLERS[key]
    assert cls.name == key and cls.label and cls.PARAMS
    for prm in cls.PARAMS:
        assert len(prm) == 7
    traj = make_trajectory("lemniscate")
    ctrl = make_controller(key, "pae_x450")
    sim = make_tracking_sim(traj, ctrl, "nominal", seed=1)
    L = sim.run(4.0)
    err = np.sqrt(sum((L[f"pos_{i}"] - L[f"ref_{i}"]) ** 2 for i in range(3)))
    assert np.all(np.isfinite(err))
    assert err.max() < 0.5, f"{key}: error maximo {err.max():.3f} m en 4 s"


def test_mpc_rl_scale_zero_equals_mpc():
    """Con residual_scale = 0 el hibrido es EXACTAMENTE el MPC."""
    from quadsim.control.position import make_controller
    from quadsim.scenarios import make_tracking_sim
    from quadsim.trajectories import make_trajectory
    logs = []
    for key, kw in (("mpc", {}), ("mpc_rl", {"residual_scale": 0.0})):
        traj = make_trajectory("circle")
        sim = make_tracking_sim(traj, make_controller(key, "pae_x450", **kw), "nominal", seed=2)
        logs.append(sim.run(2.0))
    assert np.allclose(logs[0]["pos_0"], logs[1]["pos_0"], atol=1e-12)
