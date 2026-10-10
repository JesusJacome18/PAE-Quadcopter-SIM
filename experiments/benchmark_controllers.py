"""
experiments/benchmark_controllers.py
====================================
Banco de comparacion de controladores de posicion sobre el simulador
completo (planta no lineal 6-DOF a 1 kHz, sensores con ruido y vibracion,
estimador Madgwick, lazo de actitud PID a 500 Hz, lazo externo a 100 Hz).

    controladores  x  trayectorias  x  escenarios  x  semillas

Todos los controladores ven EXACTAMENTE la misma corrida: misma semilla
(mismo ruido de sensores, misma turbulencia), misma inicializacion
(quadsim.scenarios.prepare_tracking_sim), mismo bajo nivel. Las metricas
son las de quadsim.metrics sobre la posicion verdadera.

Uso (desde la raiz del repo):
    PYTHONPATH=. python experiments/benchmark_controllers.py \
        --controllers pid mpc nmpc --trajectories lemniscate circle \
        --scenarios nominal wind --seeds 1 2 3

    --controllers   por defecto: pid mpc nmpc (+ rl, mpc_rl si se pueden importar)
    --trajectories  lemniscate circle minimum_snap step_waypoints lemniscate_fast
    --scenarios     nominal wind payload mag_emi
    --seeds         semillas (la media y desviacion se toman entre semillas)
    --estimator     estimador de actitud (mahony por defecto, igual que SimConfig; "ideal" para
                    aislar el efecto del controlador del de la estimacion)
    --out           carpeta de salida (experiments/results/benchmark)

Salidas en --out:
    runs.csv        una fila por corrida (todas las metricas + tiempos)
    summary.csv     media y desviacion por (controlador, trayectoria, escenario)
    summary.md      tabla markdown (RMSE 3D, error maximo, esfuerzo, tiempo de computo)
    xy_<traj>_<scen>.png    trayectoria XY vs referencia (semilla 1), un panel
                            por controlador, para las figuras en 8
    err_<traj>_<scen>.png   ||error|| en el tiempo, todos los controladores

Trayectoria "lemniscate_fast": la misma figura en 8 con periodo 5 s y 3
vueltas (pico 2.8 m/s y 3.95 m/s^2, cerca del limite de 5.66 m/s^2 que
impone la inclinacion maxima de 30 grados): el regimen donde la prevision y
las restricciones del MPC deberian notarse.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time
from collections import defaultdict

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from quadsim.control.position import (CONTROLLERS, UNAVAILABLE,  # noqa: E402
                                      make_controller)
from quadsim.metrics import step_summary, tracking_metrics  # noqa: E402
from quadsim.scenarios import SCENARIOS, make_tracking_sim  # noqa: E402
from quadsim.trajectories import make_trajectory  # noqa: E402

TRAJ_SPECS = {
    "lemniscate": ("lemniscate", {}),
    "circle": ("circle", {}),
    "minimum_snap": ("minimum_snap", {}),
    "step_waypoints": ("step_waypoints", {}),
    "lemniscate_fast": ("lemniscate", {"period": 5.0, "laps": 3}),
}
# colores categoricos en orden fijo (paleta de referencia del skill dataviz)
COLORS = {"pid": "#2a78d6", "mpc": "#eb6834", "nmpc": "#1baf7a",
          "rl": "#eda100", "mpc_rl": "#e87ba4"}
REF_COLOR = "#52514e"
METRICS = ["rmse_x", "rmse_y", "rmse_z", "rmse_3d", "max_err_3d", "iae", "itae",
           "effort_u", "tilt_sp_rms_deg", "thrust_std", "sat_pct"]
STEP_METRICS = ["rise_time_xy", "overshoot_pct_xy", "settling_time_xy", "ss_error_xy",
                "rise_time_z", "overshoot_pct_z", "settling_time_z", "ss_error_z"]


def run_one(ctrl_key, traj_key, scenario, seed, estimator):
    tname, tkw = TRAJ_SPECS[traj_key]
    traj = make_trajectory(tname, **tkw)
    ctrl = make_controller(ctrl_key, "pae_x450")
    sim = make_tracking_sim(traj, ctrl, scenario, seed=seed, estimator=estimator)
    n = int(round(traj.duration / sim.dt_ctrl))
    ms = []
    t0 = time.perf_counter()
    for _ in range(n):
        sim.step_control()
        if sim.k % sim.outer_every == 1:
            st = getattr(ctrl, "stats", None)
            if st and "solve_ms" in st:
                ms.append(st["solve_ms"])
    wall = time.perf_counter() - t0
    L = sim.log_arrays()
    row = {"controller": ctrl_key, "trajectory": traj_key, "scenario": scenario,
           "seed": seed, "estimator": estimator}
    row.update(tracking_metrics(L))
    if traj_key == "step_waypoints":
        row.update(step_summary(L, traj))
    ms = np.array(ms) if ms else np.array([math.nan])
    row["solve_ms_mean"] = float(np.mean(ms))
    row["solve_ms_p95"] = float(np.percentile(ms, 95)) if np.isfinite(ms).all() else math.nan
    row["solve_ms_max"] = float(np.max(ms))
    row["wall_s"] = wall
    row["realtime_factor"] = traj.duration / wall
    return row, L


def _fmt(m, s, nd=1, scale=1.0):
    if not np.isfinite(m):
        return "n/d"
    if np.isfinite(s) and s > 0:
        return f"{m * scale:.{nd}f} ± {s * scale:.{nd}f}"
    return f"{m * scale:.{nd}f}"


def summarize(rows):
    groups = defaultdict(list)
    for r in rows:
        groups[(r["controller"], r["trajectory"], r["scenario"])].append(r)
    out = []
    for (c, t, s), rs in groups.items():
        d = {"controller": c, "trajectory": t, "scenario": s, "n_seeds": len(rs)}
        keys = METRICS + ["solve_ms_mean", "solve_ms_p95", "solve_ms_max",
                          "realtime_factor"] + [k for k in STEP_METRICS if k in rs[0]]
        for k in keys:
            v = np.array([r.get(k, math.nan) for r in rs], float)
            d[k + "_mean"] = float(np.nanmean(v)) if np.any(np.isfinite(v)) else math.nan
            d[k + "_std"] = float(np.nanstd(v, ddof=1)) if np.sum(np.isfinite(v)) > 1 else math.nan
        out.append(d)
    return out


def markdown(summary, controllers, trajs, scens, estimator, seeds):
    lines = [f"# Banco de comparación de controladores de posición",
             "",
             f"PAE-X450, estimador `{estimator}`, semillas {list(seeds)} "
             f"(media ± desviación estándar muestral entre semillas, ddof = 1). "
             f"Posición verdadera. "
             f"RMSE y error máximo en cm; esfuerzo = RMS de ‖u‖ del lazo de actitud "
             f"(momento normalizado); tiempo de cómputo por llamada del lazo externo "
             f"(periodo 10 ms).",
             ""]
    idx = {(d["controller"], d["trajectory"], d["scenario"]): d for d in summary}
    for t in trajs:
        for s in scens:
            lines.append(f"## {t} — {s}")
            lines.append("")
            hdr = "| controlador | RMSE 3D [cm] | RMSE x/y/z [cm] | error máx [cm] | esfuerzo | inclin. RMS [°] | satur. [%] | cómputo medio / p95 / máx [ms] |"
            lines.append(hdr)
            lines.append("|" + "---|" * 8)
            best = min((idx[(c, t, s)]["rmse_3d_mean"] for c in controllers if (c, t, s) in idx),
                       default=math.nan)
            for c in controllers:
                d = idx.get((c, t, s))
                if d is None:
                    continue
                r3 = _fmt(d["rmse_3d_mean"], d["rmse_3d_std"], 2, 100)
                if abs(d["rmse_3d_mean"] - best) < 1e-12:
                    r3 = f"**{r3}**"
                xyz = "/".join(f"{d[f'rmse_{a}_mean'] * 100:.2f}" for a in "xyz")
                cm = (f"{d['solve_ms_mean_mean']:.2f} / {d['solve_ms_p95_mean']:.2f} / "
                      f"{d['solve_ms_max_mean']:.1f}" if np.isfinite(d["solve_ms_mean_mean"])
                      else "< 0.05 (sin QP)")
                lines.append(f"| {c} | {r3} | {xyz} | {_fmt(d['max_err_3d_mean'], d['max_err_3d_std'], 2, 100)} "
                             f"| {d['effort_u_mean']:.3f} | {d['tilt_sp_rms_deg_mean']:.1f} "
                             f"| {d['sat_pct_mean']:.2f} | {cm} |")
            if t == "step_waypoints":
                lines += ["", "Respuesta al escalón (promedio de los escalones de la caja; 2 % de banda):", "",
                          "| controlador | subida XY [s] | sobreimpulso XY [%] | establec. XY [s] | e_ss XY [cm] | subida Z [s] | sobreimpulso Z [%] | establec. Z [s] | e_ss Z [cm] |",
                          "|" + "---|" * 9]
                for c in controllers:
                    d = idx.get((c, t, s))
                    if d is None:
                        continue
                    lines.append("| " + c + " | " + " | ".join(
                        _fmt(d[f"{k}_mean"], math.nan, 2, 100 if k.startswith("ss_error") else 1)
                        for k in STEP_METRICS) + " |")
            lines.append("")
    return "\n".join(lines)


def plot_xy(logs, controllers, traj_key, scen, out_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    cs = [c for c in controllers if c in logs]
    fig, axes = plt.subplots(1, len(cs), figsize=(2.3 * len(cs) + 0.6, 5.0), sharex=True,
                             sharey=True, squeeze=False)
    for ax, c in zip(axes[0], cs):
        L = logs[c]
        # NED -> vista desde arriba con norte hacia arriba: eje horizontal = este (y)
        ax.plot(L["ref_1"], L["ref_0"], color=REF_COLOR, lw=1.2, ls="--", label="referencia")
        ax.plot(L["pos_1"], L["pos_0"], color=COLORS.get(c, "#4a3aa7"), lw=1.6, label=c)
        m = tracking_metrics(L)
        ax.set_title(f"{c}\nRMSE 3D {m['rmse_3d'] * 100:.1f} cm", fontsize=10)
        ax.set_xlabel("este y [m]")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, color="#e6e5e0", lw=0.6)
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
    axes[0][0].set_ylabel("norte x [m]")
    handles = [axes[0][0].lines[0]] + [ax.lines[1] for ax in axes[0]]
    fig.legend(handles, [h.get_label() for h in handles], loc="lower center",
               ncol=len(handles), frameon=False, fontsize=9)
    fig.suptitle(f"Seguimiento XY (vista superior): {traj_key}, {scen}, semilla 1",
                 fontsize=10)
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    p = os.path.join(out_dir, f"xy_{traj_key}_{scen}.png")
    fig.savefig(p, dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 3.2))
    for c in cs:
        L = logs[c]
        e = np.sqrt(sum((L[f"pos_{i}"] - L[f"ref_{i}"]) ** 2 for i in range(3)))
        ax.plot(L["t"], e * 100, color=COLORS.get(c, "#4a3aa7"), lw=1.2, label=c)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("‖p − p_ref‖ [cm]")
    ax.set_title(f"Error de seguimiento: {traj_key}, {scen} (semilla 1)", fontsize=10)
    ax.grid(True, color="#e6e5e0", lw=0.6)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    ax.legend(fontsize=8, frameon=False, ncol=len(cs))
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, f"err_{traj_key}_{scen}.png"), dpi=150)
    plt.close(fig)
    return p


def main(argv=None):
    default_ctrls = ["pid", "mpc", "nmpc", "rl", "mpc_rl"]
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--controllers", nargs="+", default=default_ctrls)
    ap.add_argument("--trajectories", nargs="+",
                    default=["lemniscate", "circle", "minimum_snap", "step_waypoints"],
                    choices=list(TRAJ_SPECS))
    ap.add_argument("--scenarios", nargs="+", default=list(SCENARIOS), choices=list(SCENARIOS))
    ap.add_argument("--seeds", nargs="+", type=int, default=[1])
    ap.add_argument("--estimator", default="mahony")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                  "results", "benchmark"))
    ap.add_argument("--no-plots", action="store_true")
    a = ap.parse_args(argv)
    missing = [c for c in a.controllers if c not in CONTROLLERS]
    if missing:
        # un controlador pedido que falta es un error, no un aviso: antes el
        # banco seguia sin rl/mpc_rl y la tabla salia incompleta sin notarlo
        why = "; ".join(f"{c}: {UNAVAILABLE.get(c, 'no registrado')}" for c in missing)
        raise SystemExit(f"controladores pedidos no disponibles ({why}). "
                         "Quitarlos con --controllers si es intencional.")
    ctrls = list(a.controllers)
    os.makedirs(a.out, exist_ok=True)

    rows = []
    plot_logs = defaultdict(dict)
    total = len(ctrls) * len(a.trajectories) * len(a.scenarios) * len(a.seeds)
    k = 0
    for t in a.trajectories:
        for s in a.scenarios:
            for seed in a.seeds:
                for c in ctrls:
                    k += 1
                    row, L = run_one(c, t, s, seed, a.estimator)
                    rows.append(row)
                    if seed == a.seeds[0] and t.startswith("lemniscate"):
                        plot_logs[(t, s)][c] = L
                    print(f"[{k}/{total}] {c:7s} {t:16s} {s:8s} semilla {seed}: "
                          f"RMSE3D {row['rmse_3d'] * 100:6.2f} cm  max {row['max_err_3d'] * 100:6.2f} cm  "
                          f"computo {row['solve_ms_mean']:.2f} ms  ({row['wall_s']:.1f} s)",
                          flush=True)

    keys = []
    for r in rows:
        for kk in r:
            if kk not in keys:
                keys.append(kk)
    with open(os.path.join(a.out, "runs.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    summ = summarize(rows)
    skeys = []
    for r in summ:
        for kk in r:
            if kk not in skeys:
                skeys.append(kk)
    with open(os.path.join(a.out, "summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=skeys)
        w.writeheader()
        w.writerows(summ)
    md = markdown(summ, ctrls, a.trajectories, a.scenarios, a.estimator, a.seeds)
    with open(os.path.join(a.out, "summary.md"), "w") as f:
        f.write(md + "\n")
    if not a.no_plots:
        for (t, s), logs in plot_logs.items():
            plot_xy(logs, ctrls, t, s, a.out)
    print(md)
    return rows


if __name__ == "__main__":
    main()
