"""
experiments/train_rl.py
=======================
Flujo completo del RL OFFLINE del lazo de posicion (ver docs/RL.md):

    PYTHONPATH=. python experiments/train_rl.py identify     # tau, retardo del 6-DOF (archivo versionado)
    PYTHONPATH=. python experiments/train_rl.py surrogate    # figura sustituto vs 6-DOF
    PYTHONPATH=. python experiments/train_rl.py train --mode pure     [--steps N --seed S]
    PYTHONPATH=. python experiments/train_rl.py train --mode residual [--steps N --seed S]
    PYTHONPATH=. python experiments/train_rl.py validate     # Sim2Sim en el 6-DOF completo

Salidas: models/*.npz (pesos), experiments/results/rl/*.csv|png|json.
Semillas fijas: el mismo comando da el mismo resultado en la misma maquina.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from quadsim.rl.common import MODELS_DIR  # noqa: E402

OUT = ROOT / "experiments" / "results" / "rl"
OUT.mkdir(parents=True, exist_ok=True)

MODEL_FILES = {"pure": "rl_pure.npz", "residual": "mpc_rl_residual.npz"}


# ---------------------------------------------------------------------------
def make_base():
    """Ley base del hibrido: replica vectorizada del LinearMPC (base_mpc.py)."""
    from quadsim.rl.base_mpc import make_replica
    return make_replica()


def pd_policy(obs):
    """PD + prealimentacion leyendo la observacion (referencia de cordura)."""
    from quadsim.rl.common import A_SCALE
    a = obs[:, 9:12] * 5 - 4.0 * obs[:, 0:3] - 3.0 * obs[:, 3:6] * 2 - 1.0 * obs[:, 21:24]
    return a / A_SCALE


def evaluate_env(policy_fn, mode, n=64, steps=1000, seed=12345, randomize=True, base=None,
                 init_scale=0.0, rw=None):
    """
    Costo medio y RMSE de posicion de una politica determinista en el
    sustituto (con aleatorizacion de dominio, semilla FIJA distinta de la de
    entrenamiento, arranque sobre la referencia salvo init_scale > 0).
    """
    from quadsim.rl.env import QuadTrackEnv
    env = QuadTrackEnv(n, seed=seed, mode=mode, base=base, randomize=randomize, rw=rw)
    env.init_scale = init_scale
    obs = env.reset()
    C, E, M, F = [], [], [], 0
    for _ in range(steps):
        u = policy_fn(obs)
        obs, r, fail, trunc, info = env.step(u)
        C.append(np.minimum(info["cost"], env.rw.c_max).mean())
        E.append(np.mean(np.sum(info["e_p"] ** 2, 1)))
        M.append(np.mean(np.linalg.norm(info["e_p"], axis=1)))
        F += int(fail.sum())
    return dict(cost=float(np.mean(C)), rmse_p=float(math.sqrt(np.mean(E))),
                mean_abs_p=float(np.mean(M)), fails=F)


def cmd_train(args):
    from quadsim.rl.env import QuadTrackEnv
    from quadsim.rl.ppo import PPO, PPOConfig
    from quadsim.rl.surrogate import load_params
    from dataclasses import asdict

    mode = args.mode
    base = make_base() if mode == "residual" else None
    from quadsim.rl.env import RewardConfig
    rw = RewardConfig()
    for k in ("w_pl", "w_p", "w_v", "w_a", "w_da", "w_res"):
        v = getattr(args, k)
        if v is not None:
            setattr(rw, k, v)
    env = QuadTrackEnv(args.n_envs, seed=args.seed, mode=mode, base=base, rw=rw)
    cfg = PPOConfig(total_steps=args.steps, seed=args.seed, n_steps=args.n_steps,
                    log_std_init=args.log_std_init, lr=args.lr,
                    out_gain=0.01 if mode == "pure" else 1e-3)
    agent = PPO(env.obs_dim, env.act_dim, cfg)
    tag = args.tag or mode
    csv_path = OUT / f"learning_curve_{tag}.csv"
    best = {"cost": float("inf")}
    rows = []

    # referencia: la ley base sola (residual = 0) o un PD, en el mismo entorno de evaluacion
    if mode == "residual":
        ref_eval = evaluate_env(lambda o: np.zeros((o.shape[0], 3)), mode, base=make_base(),
                                rw=env.rw)
        print(f"[base sola, residuo 0] costo={ref_eval['cost']:.4f} rmse={ref_eval['rmse_p']:.4f}")
    else:
        ref_eval = evaluate_env(pd_policy, mode, rw=env.rw)
        print(f"[PD de referencia] costo={ref_eval['cost']:.4f} rmse={ref_eval['rmse_p']:.4f}")

    def cb(ag, row):
        if row["iter"] % args.eval_every == 0 or row["steps"] >= cfg.total_steps - env.n * cfg.n_steps:
            ev = evaluate_env(lambda o: ag.policy(o, deterministic=True)[0], mode,
                              base=make_base() if mode == "residual" else None, rw=env.rw)
            row.update({f"eval_{k}": v for k, v in ev.items()})
            if ev["cost"] < best["cost"]:
                best.update(ev)
                best["iter"] = row["iter"]
                best["steps"] = row["steps"]
                pol = ag.export()
                pol.save(MODELS_DIR / f"{tag}_best.npz" if args.tag else MODELS_DIR / MODEL_FILES[mode],
                         log_std=ag.log_std, extra_meta=meta(ag, row))
        rows.append(row)
        print(" ".join(f"{k}={v:.4g}" if isinstance(v, float) else f"{k}={v}"
                       for k, v in row.items() if k in (
                           "iter", "steps", "time_s", "mean_step_reward", "mean_ep_return",
                           "std", "approx_kl", "eval_cost", "eval_rmse_p", "eval_fails")),
              flush=True)

    def meta(ag, row):
        return {"mode": mode, "obs_dim": env.obs_dim, "act_dim": 3,
                "res_scale": env.res_scale.tolist(), "seed": args.seed,
                "trained_steps": row["steps"], "train_time_s": row["time_s"],
                "surrogate": asdict(env.sp), "ppo": asdict(cfg),
                "dr": asdict(env.dr), "reward": asdict(env.rw),
                "base": base.describe() if base is not None else None}

    t0 = time.time()
    agent.train(env, callback=cb)
    wall = time.time() - t0
    keys = sorted({k for r in rows for k in r})
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow(r)
    # ultima politica (por si se quiere comparar con la mejor)
    last = (MODELS_DIR / f"{tag}_last.npz" if args.tag
            else MODELS_DIR / MODEL_FILES[mode].replace(".npz", "_last.npz"))
    agent.export().save(last, log_std=agent.log_std,
                        extra_meta=meta(agent, rows[-1]))
    summary = {"mode": mode, "tag": tag, "wall_s": wall, "steps": rows[-1]["steps"],
               "best": best, "base_only_eval": ref_eval,
               "final_mean_step_reward": rows[-1]["mean_step_reward"]}
    (OUT / f"train_summary_{tag}.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))
    plot_curve(csv_path, OUT / f"learning_curve_{tag}.png", tag, ref_eval)


def reference_lines():
    """RMSE de PD y de la replica del MPC en el MISMO entorno de evaluacion."""
    pd = evaluate_env(pd_policy, "pure")
    mpc = evaluate_env(lambda o: np.zeros((o.shape[0], 3)), "residual", base=make_base())
    return {"PD de referencia": pd["rmse_p"], "MPC lineal (réplica) solo": mpc["rmse_p"]}


def plot_curve(csv_path, png, tag, ref_eval=None, refs=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    rows = list(csv.DictReader(open(csv_path, encoding="utf-8")))
    st = np.array([float(r["steps"]) for r in rows]) / 1e6
    rew = np.array([float(r["mean_step_reward"]) for r in rows])
    # los 128 entornos se reinician casi a la vez (episodio = 1000 pasos ~ 3.9
    # iteraciones de 256): media movil de 4 iteraciones para quitar ese vaiven
    k = 4
    rew_s = np.convolve(rew, np.ones(k) / k, mode="valid")
    ev = [(float(r["steps"]) / 1e6, float(r["eval_cost"]), float(r["eval_rmse_p"]))
          for r in rows if r.get("eval_cost")]
    summ = OUT / f"train_summary_{tag}.json"
    base_cost = None
    if summ.exists():
        be = json.loads(summ.read_text()).get("base_only_eval") or {}
        base_cost = be.get("cost")
    fig, ax = plt.subplots(1, 3, figsize=(14, 3.6))
    ax[0].plot(st[k - 1:], rew_s, lw=1)
    ax[0].set_xlabel("pasos de entrenamiento [millones]")
    ax[0].set_ylabel("recompensa media por paso\n(exploración, media móvil 4 it.)")
    ax[0].set_ylim(max(rew_s.min(), -3), 0)
    ax[0].grid(alpha=.3)
    if ev:
        e = np.array(ev)
        ax[1].plot(e[:, 0], e[:, 1], "o-", ms=3, color="#2a78d6", label="política (determinista)")
        if base_cost is not None:
            ax[1].axhline(base_cost, color="k", ls="--", lw=1,
                          label=("PD de referencia" if tag == "pure" else "MPC (réplica) solo")
                          + f" ({base_cost:.3f})")
        ax[1].set_xlabel("pasos de entrenamiento [millones]")
        ax[1].set_ylabel("costo medio por paso (evaluación)")
        ax[1].set_yscale("log")
        ax[1].grid(alpha=.3, which="both")
        ax[1].legend(fontsize=8)
        ax[2].plot(e[:, 0], e[:, 2], "o-", ms=3, color="#2a78d6", label="política (determinista)")
        if refs:
            for (lab, val), ls in zip(refs.items(), ("--", ":", "-.")):
                ax[2].axhline(val, color="k", ls=ls, lw=1, label=f"{lab} ({100 * val:.1f} cm)")
        ax[2].set_xlabel("pasos de entrenamiento [millones]")
        ax[2].set_ylabel("RMSE de posición [m]\n(sustituto con DR, evaluación)")
        ax[2].set_yscale("log")
        ax[2].grid(alpha=.3, which="both")
        ax[2].legend(fontsize=8)
    fig.suptitle({"pure": "Curva de aprendizaje PPO — RL puro",
                  "residual_smooth": "PPO — residuo del híbrido MPC+RL, variante suave (A)",
                  "residual_track": "PPO — residuo del híbrido MPC+RL, variante seguimiento (B)"
                  }.get(tag, tag))
    fig.tight_layout()
    fig.savefig(png, dpi=130)
    plt.close(fig)
    print("figura:", png)


# ---------------------------------------------------------------------------
def cmd_identify(args):
    from quadsim.rl.surrogate import identify
    p, b, _, path = identify(save=True, duration=args.duration)
    print(p)
    print(b)
    print(f"guardado en {path} (models/surrogate_id.json, el vigente, no se modifica;"
          " para promoverlo, copiarlo explicitamente)")


def cmd_surrogate(args):
    """Respuesta al escalon de a_des: 6-DOF completo vs sustituto nominal."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from quadsim.rl.surrogate import collect_full_sim, load_params, simulate_lag
    from quadsim.control.position.base import PositionController
    from quadsim.simulator import PositionCommand, SimConfig, Simulation
    from quadsim.trajectories import Hover

    sp = load_params()
    out = {}
    fig, ax = plt.subplots(1, 3, figsize=(13, 3.6))
    for j, (axis, amp, tau, d) in enumerate([(0, 2.0, sp.tau_xy, sp.delay),
                                              (2, -2.0, sp.tau_z, sp.delay_z)]):
        class Step(PositionController):
            def update(self, t, nav, traj):
                a = [0.0, 0.0, 0.0]
                if t >= 1.0:
                    a[axis] = amp
                return PositionCommand(tuple(a))
        sim = Simulation(SimConfig(mode="trajectory", start_airborne=True, start_alt=5.0))
        try:
            from quadsim.scenarios import prepare_tracking_sim
            prepare_tracking_sim(sim, Hover(alt=5.0), Step())
        except ImportError:
            sim.set_position_controller(Step())
            sim.set_trajectory(Hover(alt=5.0))
        T, V = [], []
        sim.run(2.0, callback=lambda s: (T.append(s.t), V.append(s.x[3 + axis])))
        t = np.array(T)
        a6 = np.gradient(np.array(V), t)
        u = np.where(t >= 1.0 + 0.01, amp, 0.0)    # el comando entra en el lazo externo
        vv = np.array(V)
        drag = -sp.cd_over_m * vv * np.abs(vv)
        asg = simulate_lag(t, u[:, None], tau, d)[:, 0] + drag
        ax[j].plot(t - 1.0, a6 / amp, label="6-DOF completo (Madgwick)")
        ax[j].plot(t - 1.0, asg / amp, "--", label=f"sustituto: τ={tau*1e3:.0f} ms, d={d*1e3:.0f} ms")
        ax[j].set_xlim(-0.05, 0.6)
        ax[j].set_xlabel("t − t_escalón [s]")
        ax[j].set_ylabel("a / a_des")
        ax[j].set_title(["horizontal (x)", "vertical (z)"][j] + f", escalón {abs(amp):.0f} m/s²")
        ax[j].grid(alpha=.3)
        ax[j].legend(fontsize=8)
        m = (t >= 1.0) & (t <= 1.6)
        out[["xy", "z"][j]] = {"rmse_rel": float(np.sqrt(np.mean((a6[m] - asg[m]) ** 2)) / abs(amp))}
    # trazo de la identificacion con excitacion aleatoria
    data = collect_full_sim(duration=8.0, seed=11)
    t = data["t"]
    v = data["v"]
    pred = simulate_lag(t, data["a_cmd"][:, :1], sp.tau_xy, sp.delay)[:, 0] \
        - sp.cd_over_m * v[:, 0] * np.abs(v[:, 0])
    m = (t > 2) & (t < 6)
    ax[2].plot(t[m], data["a_real"][m, 0], lw=1, label="6-DOF completo")
    ax[2].plot(t[m], pred[m], "--", lw=1, label="sustituto")
    ax[2].plot(t[m], data["a_cmd"][m, 0], ":", lw=.8, color="gray", label="a_des")
    r2 = 1 - np.mean((pred[m] - data["a_real"][m, 0]) ** 2) / np.var(data["a_real"][m, 0])
    ax[2].set_title(f"excitación NO usada en el ajuste, R²={r2:.3f}", fontsize=10)
    ax[2].set_xlabel("t [s]")
    ax[2].set_ylabel("a_x [m/s²]")
    ax[2].legend(fontsize=8)
    ax[2].grid(alpha=.3)
    out["validation_r2_x"] = float(r2)
    fig.tight_layout()
    fig.savefig(OUT / "surrogate_vs_6dof.png", dpi=130)
    (OUT / "surrogate_vs_6dof.json").write_text(json.dumps(out, indent=2))
    print(out)


def cmd_replot(args):
    refs = reference_lines()
    for tag in ("pure", "residual_smooth", "residual_track"):
        c = OUT / f"learning_curve_{tag}.csv"
        if c.exists():
            plot_curve(c, OUT / f"learning_curve_{tag}.png", tag, refs=refs)


def cmd_validate(args):
    from quadsim.rl.validation import run_validation
    run_validation(OUT, quick=args.quick)


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    a = sub.add_parser("identify")
    a.add_argument("--duration", type=float, default=30.0)
    sub.add_parser("surrogate")
    sub.add_parser("replot")
    t = sub.add_parser("train")
    t.add_argument("--mode", choices=["pure", "residual"], required=True)
    t.add_argument("--steps", type=int, default=20_000_000)
    t.add_argument("--n-envs", type=int, default=128)
    t.add_argument("--n-steps", type=int, default=256)
    t.add_argument("--seed", type=int, default=0)
    t.add_argument("--lr", type=float, default=3e-4)
    t.add_argument("--log-std-init", type=float, default=-1.0)
    t.add_argument("--eval-every", type=int, default=10)
    t.add_argument("--tag", default=None)
    for k in ("w_pl", "w_p", "w_v", "w_a", "w_da", "w_res"):
        t.add_argument("--" + k.replace("_", "-"), dest=k, type=float, default=None,
                       help="peso de la recompensa (ver quadsim/rl/env.py RewardConfig)")
    v = sub.add_parser("validate")
    v.add_argument("--quick", action="store_true")
    args = ap.parse_args()
    {"identify": cmd_identify, "surrogate": cmd_surrogate, "train": cmd_train,
     "validate": cmd_validate, "replot": cmd_replot}[args.cmd](args)


if __name__ == "__main__":
    main()
