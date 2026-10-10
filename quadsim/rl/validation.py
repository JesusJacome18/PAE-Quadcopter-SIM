"""
quadsim.rl.validation
=====================
Paso de VALIDACION del flujo offline -> Sim2Real (Sim2Sim) -> validacion.

1. Banco 6-DOF con el MISMO protocolo de experiments/benchmark_controllers.py
   (mismas funciones: make_trajectory con TRAJ_SPECS, make_controller,
   scenarios.make_tracking_sim, metrics.tracking_metrics sobre la posicion
   verdadera, misma duracion): PID, MPC, RL, MPC+RL (y la variante "suave"
   del residuo) en figura en 8, circulo y figura en 8 rapida, escenarios
   nominal, viento y carga (+300 g en t = 8 s), semillas 1-3. Se usa un
   bucle propio (y no el script) solo para poder pasar parametros a los
   controladores (variante del residuo).
2. Brecha sustituto -> 6-DOF ("reality gap" de Sim2Sim): la misma
   trayectoria y el mismo controlador en el entorno de entrenamiento
   NOMINAL (sin aleatorizar, sin ruido) y en el 6-DOF nominal.
3. Ablacion del hibrido: residual_scale in {0, 0.5, 1}; 0 debe reproducir
   el MPC exactamente.
"""

from __future__ import annotations

import csv
import json
import math
import sys
import time
from collections import defaultdict
from pathlib import Path

import numpy as np

from .common import REPO_ROOT

TRAJS = ["lemniscate", "circle", "lemniscate_fast"]
SCENS = ["nominal", "wind", "payload"]
VARIANTS = [("pid", "pid", {}), ("mpc", "mpc", {}), ("rl", "rl", {}),
            ("mpc_rl", "mpc_rl", {}), ("mpc_rl_suave", "mpc_rl", {"model_idx": 1})]
COLORS = {"pid": "#2a78d6", "mpc": "#eb6834", "rl": "#eda100", "mpc_rl": "#e87ba4",
          "mpc_rl_suave": "#8f5bd6"}
LABELS = {"pid": "PID", "mpc": "MPC lineal", "rl": "RL puro", "mpc_rl": "MPC + RL residual",
          "mpc_rl_suave": "MPC + RL residual (suave)"}


def _traj_specs():
    sys.path.insert(0, str(REPO_ROOT / "experiments"))
    from benchmark_controllers import TRAJ_SPECS
    return TRAJ_SPECS


def run_full(ctrl_key: str, traj_key: str, scenario="nominal", seed=1, **params):
    from ..control.position import make_controller
    from ..metrics import tracking_metrics
    from ..scenarios import make_tracking_sim
    from ..trajectories import make_trajectory
    tname, tkw = _traj_specs()[traj_key]
    traj = make_trajectory(tname, **tkw)
    ctrl = make_controller(ctrl_key, "pae_x450", **params)
    sim = make_tracking_sim(traj, ctrl, scenario, seed=seed)
    t0 = time.perf_counter()
    L = sim.run(traj.duration)
    wall = time.perf_counter() - t0
    m = tracking_metrics(L)
    m["wall_s"] = wall
    m["realtime_factor"] = traj.duration / wall
    # el estimador que realmente volo (antes el resumen decia "Madgwick" fijo
    # aunque SimConfig ya usaba Mahony por defecto)
    m["estimator"] = sim.estimator.name
    return m, L


def _policy_surrogate(policy):
    """SurrogateParams con los que se ENTRENO la politica (meta['surrogate']);
    si el archivo no los trae, los vigentes (models/surrogate_id.json)."""
    from .surrogate import SurrogateParams, load_params
    sp = (policy.meta or {}).get("surrogate")
    if not sp:
        return load_params()
    keys = SurrogateParams.__dataclass_fields__.keys()
    return SurrogateParams(**{k: v for k, v in sp.items() if k in keys})


def surrogate_rmse(ctrl_key: str, traj, **params) -> float:
    """
    RMSE 3D [m] en el sustituto nominal (sin DR ni ruido), 1 entorno.
    Cada politica se evalua en el sustituto con que se ENTRENO (guardado en
    sus metadatos): la politica pura se entreno con una identificacion
    anterior (tau 90 ms / retardo 28 ms) y no con la vigente de
    models/surrogate_id.json. El MPC solo se evalua con la vigente.
    """
    from .base_mpc import make_replica
    from .env import QuadTrackEnv
    from .policy import MLPPolicy
    if ctrl_key == "rl":
        pol = MLPPolicy.load("rl_pure.npz")
        env = QuadTrackEnv(1, mode="pure", randomize=False, fixed_traj=traj,
                           sp=_policy_surrogate(pol))
        fn = pol.act
    else:
        pol = None
        if ctrl_key != "mpc":
            from ..control.position.mpc_rl import MODEL_FILES
            pol = MLPPolicy.load(MODEL_FILES[int(params.get("model_idx", 0))])
        env = QuadTrackEnv(1, mode="residual", base=make_replica(), randomize=False,
                           fixed_traj=traj,
                           sp=_policy_surrogate(pol) if pol is not None else None)
        if pol is None:
            fn = lambda o: np.zeros((o.shape[0], 3))       # noqa: E731
        else:
            fn = pol.act
    env.init_scale = 0.0
    obs = env.reset()
    E = []
    for _ in range(env.ep_len - 1):
        obs, r, fail, trunc, info = env.step(fn(obs))
        E.append(float(np.sum(info["e_p"] ** 2)))
        if fail[0]:
            return float("nan")
    return math.sqrt(np.mean(E))


def _fmt(v, s=None, scale=100.0, nd=2):
    if not np.isfinite(v):
        return "n/d"
    if s is not None and np.isfinite(s) and s > 0:
        return f"{v * scale:.{nd}f} ± {s * scale:.{nd}f}"
    return f"{v * scale:.{nd}f}"


def run_validation(out: Path, quick: bool = False):
    out = Path(out)
    vdir = out / "validation"
    vdir.mkdir(parents=True, exist_ok=True)
    seeds = [1] if quick else [1, 2, 3]
    rows = []
    logs = {}
    # ---- 1. banco 6-DOF -----------------------------------------------------
    for tk in TRAJS:
        for sc in SCENS:
            for seed in seeds:
                for vk, ck, prm in VARIANTS:
                    m, L = run_full(ck, tk, sc, seed, **prm)
                    row = {"variant": vk, "trajectory": tk, "scenario": sc, "seed": seed}
                    row.update(m)
                    rows.append(row)
                    if seed == seeds[0]:
                        logs[(vk, tk, sc)] = {k: L[k] for k in
                                              ("t", "pos_0", "pos_1", "pos_2", "ref_0", "ref_1", "ref_2")}
                    print(f"{vk:13s} {tk:16s} {sc:8s} s{seed}  RMSE3D={100 * m['rmse_3d']:.2f} cm"
                          f"  max={100 * m['max_err_3d']:.2f} cm  ({m['wall_s']:.1f} s)", flush=True)
    keys = list(rows[0].keys())
    with open(vdir / "runs.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    G = defaultdict(list)
    for r in rows:
        G[(r["variant"], r["trajectory"], r["scenario"])].append(r)
    summ = {}
    for k, rs in G.items():
        d = {}
        for mk in ("rmse_3d", "rmse_x", "rmse_y", "rmse_z", "max_err_3d", "effort_u",
                   "tilt_sp_rms_deg", "realtime_factor"):
            v = np.array([r[mk] for r in rs], float)
            # desviacion estandar MUESTRAL (ddof = 1), la misma que
            # experiments/benchmark_controllers.py: antes aqui era poblacional
            # (ddof = 0) y los +- de ambos resumenes no eran comparables
            d[mk] = (float(v.mean()), float(v.std(ddof=1)) if len(v) > 1 else float("nan"))
        summ["/".join(k)] = d
    est_names = sorted({str(r.get("estimator", "?")) for r in rows})
    md = ["# Validación Sim2Sim de los controladores de RL (simulador 6-DOF completo)", "",
          f"PAE-X450, estimador {', '.join(est_names)}, semillas {seeds} "
          "(media ± desviación estándar muestral entre semillas, ddof = 1), "
          "posición verdadera, mismo protocolo que `experiments/benchmark_controllers.py`. "
          "RMSE y error máximo en cm; esfuerzo = RMS de ‖u‖ del lazo de actitud.", ""]
    for tk in TRAJS:
        for sc in SCENS:
            md += [f"## {tk} — {sc}", "",
                   "| controlador | RMSE 3D [cm] | RMSE x/y/z [cm] | error máx [cm] | esfuerzo | inclin. RMS [°] |",
                   "|---|---|---|---|---|---|"]
            best = min(summ[f"{v}/{tk}/{sc}"]["rmse_3d"][0] for v, _, _ in VARIANTS)
            for vk, _, _ in VARIANTS:
                d = summ[f"{vk}/{tk}/{sc}"]
                r3 = _fmt(*d["rmse_3d"])
                if d["rmse_3d"][0] == best:
                    r3 = f"**{r3}**"
                md.append(f"| {vk} | {r3} | {_fmt(d['rmse_x'][0])}/{_fmt(d['rmse_y'][0])}/"
                          f"{_fmt(d['rmse_z'][0])} | {_fmt(*d['max_err_3d'])} | "
                          f"{d['effort_u'][0]:.3f} | {d['tilt_sp_rms_deg'][0]:.1f} |")
            md.append("")
    (vdir / "summary.md").write_text("\n".join(md), encoding="utf-8")
    (vdir / "summary.json").write_text(json.dumps(summ, indent=1), encoding="utf-8")
    _plots(logs, vdir)

    # ---- 2. brecha sustituto -> 6-DOF -------------------------------------
    from ..trajectories import make_trajectory
    gap = {}
    for tk in TRAJS:
        tname, tkw = _traj_specs()[tk]
        for vk, ck, prm in VARIANTS[1:]:
            s = surrogate_rmse(ck, make_trajectory(tname, **tkw), **prm)
            f = summ[f"{vk}/{tk}/nominal"]["rmse_3d"][0]
            gap[f"{vk}/{tk}"] = {"surrogate_rmse_cm": 100 * s, "full6dof_rmse_cm": 100 * f}
            print("brecha", vk, tk, gap[f"{vk}/{tk}"], flush=True)
    # ---- 3. ablacion: escala del residuo ------------------------------------
    abl = {}
    for tk in ("lemniscate", "lemniscate_fast"):
        for s in (0.0, 0.5, 1.0):
            abl[f"{tk}/scale{s}"] = 100 * run_full("mpc_rl", tk, "nominal", 1,
                                                   residual_scale=s)[0]["rmse_3d"]
        abl[f"{tk}/mpc"] = 100 * summ[f"mpc/{tk}/nominal"]["rmse_3d"][0] if seeds == [1] \
            else 100 * run_full("mpc", tk, "nominal", 1)[0]["rmse_3d"]
    print("ablacion", abl, flush=True)
    (out / "validation_gap_ablation.json").write_text(
        json.dumps({"gap_nominal_seed_mean": gap, "ablation_nominal_seed1_cm": abl}, indent=2))


def _plots(logs, vdir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    for tk in TRAJS:
        # trayectoria XY en nominal, un panel por controlador
        fig, axs = plt.subplots(1, len(VARIANTS), figsize=(3.2 * len(VARIANTS), 3.4),
                                sharex=True, sharey=True)
        for ax, (vk, _, _) in zip(axs, VARIANTS):
            L = logs[(vk, tk, "nominal")]
            ax.plot(L["ref_1"], L["ref_0"], color="#52514e", lw=1, ls="--", label="referencia")
            ax.plot(L["pos_1"], L["pos_0"], color=COLORS[vk], lw=1.2, label=LABELS[vk])
            ax.set_title(LABELS[vk], fontsize=9)
            ax.set_xlabel("y (este) [m]")
            ax.set_aspect("equal")
            ax.grid(alpha=.3)
        axs[0].set_ylabel("x (norte) [m]")
        fig.suptitle(f"{tk} — nominal (6-DOF completo)")
        fig.tight_layout()
        fig.savefig(vdir / f"xy_{tk}_nominal.png", dpi=120)
        plt.close(fig)
        # error en el tiempo, los 3 escenarios
        fig, axs = plt.subplots(1, len(SCENS), figsize=(13, 3.4), sharey=True)
        for ax, sc in zip(axs, SCENS):
            for vk, _, _ in VARIANTS:
                L = logs[(vk, tk, sc)]
                e = np.sqrt(sum((L[f"pos_{i}"] - L[f"ref_{i}"]) ** 2 for i in range(3)))
                ax.plot(L["t"], 100 * e, color=COLORS[vk], lw=1, label=LABELS[vk])
            ax.set_title(sc)
            ax.set_xlabel("t [s]")
            ax.grid(alpha=.3)
        axs[0].set_ylabel("‖error de posición‖ [cm]")
        axs[-1].legend(fontsize=7)
        fig.suptitle(f"{tk} (semilla 1)")
        fig.tight_layout()
        fig.savefig(vdir / f"err_{tk}.png", dpi=120)
        plt.close(fig)
