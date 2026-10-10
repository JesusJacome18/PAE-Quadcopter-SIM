"""
Figuras de los documentos de registro (docs/registro/figuras/).

    PYTHONPATH=. python docs/registro/figuras.py            # todas
    PYTHONPATH=. python docs/registro/figuras.py --sin-sim  # solo diagramas

* Diagramas de bloques dibujados con matplotlib (arquitectura en capas, lazo de
  control, relojes, hilos y procesos, flujo de sintonía).
* Figura de portada: corrida REAL del simulador (MPC lineal siguiendo la figura
  en 8 con el PAE-X450, semilla 1); no es una ilustración.
* Figuras del documento del modelo: se rasterizan a 300 ppp desde sus PDF
  vectoriales (docs/modelo/figuras/*.pdf) para que se impriman nítidas.
"""

from __future__ import annotations

import argparse
import glob
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
OUT = os.path.join(HERE, "figuras")
sys.path.insert(0, REPO)

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Rectangle

NAVY = "#0B2341"
NAVY2 = "#16365F"
CYAN = "#00B8D4"
CYAN_D = "#00838F"
CYAN_L = "#E4F7FA"
MAG = "#C8157F"
MAG_L = "#FBEAF3"
GRAY = "#5B6B7B"
RULE = "#C5D0DB"
ZEBRA = "#F2F6F9"
AMBER = "#B26A00"
AMBER_L = "#FFF4E0"

for fam in ("Carlito", "Calibri", "DejaVu Sans"):
    try:
        from matplotlib import font_manager
        font_manager.findfont(fam, fallback_to_default=False)
        plt.rcParams["font.family"] = fam
        break
    except Exception:  # noqa: BLE001
        continue
plt.rcParams.update({"font.size": 9, "axes.edgecolor": GRAY, "axes.labelcolor": NAVY,
                     "xtick.color": GRAY, "ytick.color": GRAY, "savefig.dpi": 300})

CM = 1 / 2.54


def box(ax, x, y, w, h, title, lines=(), fc="white", ec=NAVY, tc=NAVY, lw=1.2, title_size=9.5,
        line_size=7.6, round_=0.02, mono=False, lc=GRAY, bold=True):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle=f"round,pad=0,rounding_size={round_}",
                                fc=fc, ec=ec, lw=lw, zorder=2))
    n = len(lines)
    ty = y + h / 2 + (n * 0.5) * (line_size / 72 * 1.25) * 0  # base
    total = 1 + n
    step = h / (total + 1)
    ax.text(x + w / 2, y + h - step * 1.0, title, ha="center", va="center",
            fontsize=title_size, color=tc, fontweight="bold" if bold else "normal", zorder=3)
    for i, l in enumerate(lines):
        ax.text(x + w / 2, y + h - step * (2.0 + i * 0.95), l, ha="center", va="center",
                fontsize=line_size, color=lc, zorder=3,
                family="monospace" if mono else None)


def arrow(ax, x0, y0, x1, y1, color=NAVY, lw=1.2, style="-|>", text=None, tx=0, ty=0,
          ls="-", rad=0.0, fs=7.2, tcolor=None):
    a = FancyArrowPatch((x0, y0), (x1, y1), arrowstyle=style, mutation_scale=9, color=color,
                        lw=lw, linestyle=ls, connectionstyle=f"arc3,rad={rad}", zorder=1,
                        shrinkA=0, shrinkB=0)
    ax.add_patch(a)
    if text:
        ax.text((x0 + x1) / 2 + tx, (y0 + y1) / 2 + ty, text, ha="center", va="center",
                fontsize=fs, color=tcolor or color, zorder=4,
                bbox=dict(fc="white", ec="none", pad=0.6))


def canvas(w_cm, h_cm, xlim=(0, 100), ylim=(0, 100)):
    fig = plt.figure(figsize=(w_cm * CM, h_cm * CM))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.axis("off")
    return fig, ax


def save(fig, name):
    os.makedirs(OUT, exist_ok=True)
    p = os.path.join(OUT, name)
    fig.savefig(p, dpi=300, facecolor="white")
    plt.close(fig)
    print("figura:", p)
    return p


# --------------------------------------------------------------- diagramas
def fig_capas():
    """Arquitectura en capas: presentación / coordinación / núcleo / experimentos."""
    fig, ax = canvas(16.8, 11.2)
    bands = [
        (77, 22, "PRESENTACIÓN", "gui/  ·  main.py  (PySide6 + pyqtgraph + PyOpenGL)"),
        (56, 19, "COORDINACIÓN", "gui/controller.py · gui/store.py · gui/workers.py"),
        (13, 41, "NÚCLEO DE SIMULACIÓN", "quadsim/  (solo numpy; sin dependencias de interfaz)"),
        (0.5, 11, "EXPERIMENTOS Y DATOS", "experiments/ · tests/ · models/ · docs/"),
    ]
    for y, h, t, sub in bands:
        ax.add_patch(Rectangle((0.5, y), 99, h, fc=ZEBRA, ec="none", zorder=0))
        tt = ax.text(1.6, y + h - 2.4, t, fontsize=7.5, color=CYAN_D, fontweight="bold",
                     va="center")
        ax.text(1.6 + 1.25 * len(t) + 1.5, y + h - 2.4, "·  " + sub, fontsize=7, color=GRAY,
                va="center", ha="left")
    pages = ["Actitud", "Trayectorias", "Perturbaciones", "Sensores", "Barridos", "Modelo"]
    for i, p in enumerate(pages):
        box(ax, 2 + i * 13.2, 79, 12.4, 13, p, ["gui/pages"], fc="white", ec=NAVY2,
            title_size=8, line_size=6.5)
    box(ax, 81.6, 79, 16.6, 13, "Instrumentos", ["horizonte · brújula", "motores · viento · 3D"],
        title_size=8, line_size=6.3, ec=NAVY2)
    box(ax, 2, 58, 40, 12.5, "SimulationController", ["dueño único de la Simulation",
                                                      "señales Qt · QTimer 50 Hz"],
        fc=CYAN_L, ec=CYAN_D, title_size=8.5, line_size=6.8)
    box(ax, 47, 58, 22, 12.5, "TelemetryStore", ["buffers circulares", "100 Hz · giro 500 Hz"],
        fc=CYAN_L, ec=CYAN_D, title_size=8.5, line_size=6.8)
    box(ax, 74, 58, 24.2, 12.5, "Workers (QThread)", ["CompareWorker", "SweepWorker"],
        fc=CYAN_L, ec=CYAN_D, title_size=8.5, line_size=6.8)
    box(ax, 30, 38, 40, 12.5, "simulator.Simulation",
        ["SimConfig · relojes 1 kHz / 500 Hz / 100 Hz", "telemetry() · run() · advance()"],
        fc="white", ec=MAG, tc=MAG, lw=1.6, title_size=9, line_size=6.8)
    mods = [
        (2, 42, "dynamics", "6-DOF · RK4"), (2, 31, "propulsion", "ESC · mezclador"),
        (2, 20, "vehicles", "catálogo · fuentes"), (16, 20, "math3d", "cuaterniones · NED"),
        (16, 42, "control/tuning", "SIMC"), (16, 31, "identification", "rezagos FOPDT"),
        (72, 42, "sensors", "IMU · mag · baro"), (86, 42, "filters", "biquad · PT1"),
        (72, 31, "estimators", "AHRS · navegación"), (86, 31, "disturbances", "viento · EMI · carga"),
        (30, 28, "control/attitude", "PID ángulo→tasa"), (51, 28, "control/position", "PID·MPC·NMPC·RL"),
        (30, 18, "trajectories", "7 generadores"), (51, 18, "rl", "PPO · entorno rápido"),
        (72, 20, "sweeps", "objectives · barridos"), (86, 20, "metrics", "scenarios · métricas"),
    ]
    for x, y, t, s_ in mods:
        w = 19 if x in (30, 51) else 12.4
        h = 8.4 if x in (30, 51) else 9
        box(ax, x, y, w, h, t, [s_], title_size=7.4, line_size=6.0, ec=NAVY2)
    exps = ["benchmark_controllers.py", "sweep_cli.py", "train_rl.py", "model_verification.py",
            "tests/ (pytest)"]
    for i, e in enumerate(exps):
        box(ax, 2 + i * 19.4, 1.6, 18.6, 5.6, e, [], title_size=7, ec=GRAY, tc=NAVY2, bold=False)
    arrow(ax, 15, 79, 15, 70.7, color=NAVY)
    ax.text(16, 75, "métodos", fontsize=6.8, color=NAVY, va="center")
    arrow(ax, 30, 70.7, 30, 79, color=CYAN_D)
    ax.text(31, 75, "señales", fontsize=6.8, color=CYAN_D, va="center")
    arrow(ax, 42, 64.2, 47, 64.2, color=CYAN_D)
    arrow(ax, 58, 70.7, 58, 79, color=CYAN_D)
    ax.text(59, 75, "lectura para redibujo", fontsize=6.8, color=CYAN_D, va="center")
    arrow(ax, 22, 58, 38, 50.7, color=NAVY)
    arrow(ax, 86, 58, 66, 50.7, color=NAVY, text="copias de SimConfig", ty=1.5, tx=4)
    arrow(ax, 50, 7.4, 50, 17.8, color=GRAY, ls="--")
    ax.text(51, 10.6, "importan quadsim (sin GUI)", fontsize=6.8, color=GRAY, va="center")
    return save(fig, "d1_arquitectura_capas.png")


def fig_lazo():
    """Lazo de control completo con sus tasas."""
    fig, ax = canvas(16.8, 8.2)
    # zonas de tasa
    zones = [(1, 52, 23, 40, "LAZO EXTERNO · 100 Hz", CYAN_L, CYAN_D),
             (26, 52, 39, 40, "LAZO DE CONTROL · 500 Hz", ZEBRA, NAVY2),
             (67, 52, 32, 40, "FÍSICA · 1 kHz · RK4", MAG_L, MAG)]
    for x, y, w, h, t, fc, ec in zones:
        ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=1.2",
                                    fc=fc, ec=ec, lw=0.8, ls="--", zorder=0))
        ax.text(x + 1.2, y + h - 3.5, t, fontsize=7, color=ec, fontweight="bold")
    box(ax, 3, 62, 19, 18, "Control de posición", ["PID · MPC · NMPC", "RL · MPC+RL"],
        title_size=7.8, line_size=6.6, ec=CYAN_D)
    box(ax, 28, 62, 15, 18, "Planitud", ["diferencial", "a_des → T, φ, θ"], title_size=8,
        line_size=6.6)
    box(ax, 45, 62, 18, 18, "PID ángulo→tasa", ["P ángulo · PID tasa", "u en [−1, 1]"],
        title_size=8, line_size=6.6)
    box(ax, 69, 62, 13, 18, "Mezclador", ["desaturación", "por prioridades"], title_size=8,
        line_size=6.4)
    box(ax, 84, 62, 13.5, 18, "Planta 6-DOF", ["ESC + motor", "Newton–Euler"], title_size=8,
        line_size=6.4, ec=MAG, tc=MAG)
    arrow(ax, 22, 71, 28, 71, text="a_des", ty=3.2)
    arrow(ax, 43, 71, 45, 71)
    arrow(ax, 63, 71, 69, 71, text="u", ty=3.2)
    arrow(ax, 82, 71, 84, 71, text="σ", ty=3.2)
    # retroalimentación
    box(ax, 69, 18, 28.5, 13, "Sensores", ["IMU (vibración 177 Hz) · magnetómetro", "barómetro · posición externa"],
        title_size=8, line_size=6.4, ec=NAVY2)
    box(ax, 38, 18, 26, 13, "Filtros + estimadores", ["biquad · Madgwick · Mahony · EKF",
                                                      "Kalman de navegación"],
        title_size=8, line_size=6.4, ec=NAVY2)
    box(ax, 3, 18, 19, 13, "Trayectoria", ["7 generadores", "derivadas analíticas"],
        title_size=8, line_size=6.4, ec=MAG, tc=MAG)
    arrow(ax, 90.7, 62, 90.7, 31, text="estado verdadero", tx=0, ty=0)
    arrow(ax, 69, 24.5, 64, 24.5)
    arrow(ax, 51, 31, 51, 62, color=CYAN_D, text="actitud estimada", ty=4)
    arrow(ax, 41, 31, 15, 62, color=CYAN_D, text="posición y velocidad\nestimadas", tx=-3, ty=-3)
    arrow(ax, 12.5, 31, 12.5, 62, color=MAG, text="ref.", tx=0, ty=-8)
    box(ax, 84, 1.5, 13.5, 9.5, "Perturbaciones", ["viento · carga · falla"], title_size=7.4,
        line_size=6.2, ec=AMBER, tc=AMBER, fc=AMBER_L)
    arrow(ax, 95, 11, 95, 62, color=AMBER, ls="--")
    box(ax, 66, 1.5, 15, 9.5, "EMI", ["magnetómetro"], title_size=7.4, line_size=6.2, ec=AMBER,
        tc=AMBER, fc=AMBER_L)
    arrow(ax, 76, 11, 76, 18, color=AMBER, ls="--")
    return save(fig, "d2_lazo_control.png")


def fig_relojes():
    """Diagrama de tiempos: 20 ms de los cuatro relojes."""
    fig = plt.figure(figsize=(16.8 * CM, 5.4 * CM))
    ax = fig.add_axes([0.2, 0.2, 0.77, 0.72])
    rows = [("Física (RK4)", 1.0, MAG), ("Control: IMU, AHRS, filtros, PID", 2.0, NAVY),
            ("Lazo externo (posición)", 10.0, CYAN_D), ("Dibujo de la GUI", 20.0, GRAY)]
    for i, (lab, per, col) in enumerate(rows):
        y = len(rows) - 1 - i
        t = 0.0
        while t <= 20.0 + 1e-9:
            ax.plot([t, t], [y - 0.32, y + 0.32], color=col, lw=1.6 if per > 1 else 1.0)
            t += per
        ax.text(-0.6, y, lab, ha="right", va="center", fontsize=7.6, color=NAVY)
        hz = 1000.0 / per
        ax.text(20.6, y, f"{hz:g} Hz", ha="left", va="center", fontsize=7.6, color=col,
                fontweight="bold")
    ax.set_xlim(-0.3, 20.3)
    ax.set_ylim(-0.7, len(rows) - 0.3)
    ax.set_yticks([])
    for s in ("left", "right", "top"):
        ax.spines[s].set_visible(False)
    ax.set_xticks(range(0, 21, 2))
    ax.set_xlabel("Tiempo simulado [ms]", fontsize=8)
    ax.tick_params(labelsize=7)
    ax.grid(axis="x", color=RULE, lw=0.5)
    return save(fig, "d3_relojes.png")


def fig_hilos():
    """Hilo de la GUI, hilos de trabajo y procesos de barrido."""
    fig, ax = canvas(16.8, 8.0)
    ax.add_patch(Rectangle((1, 52), 98, 44, fc=CYAN_L, ec="none"))
    ax.add_patch(Rectangle((1, 2), 47, 44, fc=ZEBRA, ec="none"))
    ax.add_patch(Rectangle((51, 2), 48, 44, fc=MAG_L, ec="none"))
    ax.text(2.2, 92, "HILO PRINCIPAL (Qt)", fontsize=7.2, color=CYAN_D, fontweight="bold")
    ax.text(2.2, 42, "HILOS DE TRABAJO (QThread)", fontsize=7.2, color=NAVY2, fontweight="bold")
    ax.text(52.2, 42, "PROCESOS (multiprocessing, spawn; solo si procesos > 1)", fontsize=7.2,
            color=MAG, fontweight="bold")
    box(ax, 3, 58, 22, 27, "SimulationController", ["QTimer 50 Hz", "presupuesto 28 ms/tic"],
        title_size=7.8, line_size=6.5, ec=CYAN_D)
    box(ax, 29, 58, 22, 27, "Simulation", ["pasos de control 2 ms", "2 subpasos RK4"],
        title_size=8, line_size=6.5)
    box(ax, 55, 58, 18, 27, "TelemetryStore", ["100 Hz · giro 500 Hz"], title_size=8,
        line_size=6.5)
    box(ax, 77, 58, 20, 27, "Página visible", ["redibujo ≤ 30 Hz", "setData()"], title_size=8,
        line_size=6.5)
    arrow(ax, 25, 71.5, 29, 71.5)
    arrow(ax, 51, 71.5, 55, 71.5)
    arrow(ax, 73, 71.5, 77, 71.5)
    box(ax, 3, 7, 20, 27, "CompareWorker", ["controladores en serie", "sin GUI"],
        title_size=8, line_size=6.5, ec=NAVY2)
    box(ax, 26, 7, 20, 27, "SweepWorker", ["run_sweep()", "progreso · cancelar"],
        title_size=8, line_size=6.5, ec=NAVY2)
    for i in range(3):
        box(ax, 55 + i * 14.5, 9, 12.5, 25, f"Proceso {i + 1}", ["una corrida", "(punto, semilla)"],
            title_size=7.6, line_size=6.0, ec=MAG, tc=MAG)
    ax.text(97.5, 4.2, "cada corrida se agrega a runs.jsonl (reanudable)", fontsize=6.6, color=MAG,
            ha="right")
    arrow(ax, 14, 58, 14, 34, color=NAVY2)
    ax.text(15, 49, "snapshot_config()", fontsize=6.6, color=NAVY2, va="center")
    arrow(ax, 46, 20, 55, 20, color=MAG)
    arrow(ax, 36, 34, 87, 58, color=CYAN_D, ls="--", rad=-0.12)
    ax.text(66, 48.5, "progreso y resultados (señales)", fontsize=6.6, color=CYAN_D,
            ha="center", va="center", bbox=dict(fc=MAG_L, ec="none", pad=0.4))
    return save(fig, "d4_hilos_procesos.png")


def fig_flujo():
    """Flujo de trabajo de sintonía: del modelo al dron real."""
    fig, ax = canvas(16.8, 4.6)
    steps = [("1 · Vehículo", ["parámetros con fuente", "catálogo / datos medidos"]),
             ("2 · SIMC", ["ganancias iniciales", "desde el modelo"]),
             ("3 · Barridos", ["filtros, estimador, PID", "costo J · Pareto"]),
             ("4 · Banco", ["5 controladores", "mismas semillas"]),
             ("5 · Dron real", ["punto de partida", "de la sintonía"])]
    w, gap = 17.2, 2.9
    for i, (t, ls) in enumerate(steps):
        x = 1 + i * (w + gap)
        last = i == len(steps) - 1
        box(ax, x, 18, w, 64, t, ls, fc=MAG_L if last else (CYAN_L if i else "white"),
            ec=MAG if last else (CYAN_D if i else NAVY), tc=MAG if last else NAVY,
            title_size=8.5, line_size=6.8)
        if not last:
            arrow(ax, x + w + 0.2, 50, x + w + gap - 0.2, 50, color=NAVY, lw=1.4)
    ax.text(50, 6, "Pasos 1–4 dentro de QuadSIM-PRIMBIO; el paso 5 queda fuera del software.",
            ha="center", fontsize=7, color=GRAY)
    return save(fig, "d5_flujo_sintonia.png")


# --------------------------------------------------------------- portada
def fig_portada():
    """Corrida real: MPC lineal sobre la figura en 8 (PAE-X450, semilla 1)."""
    import numpy as np
    from quadsim.control.position import make_controller
    from quadsim.scenarios import make_tracking_sim
    from quadsim.trajectories import make_trajectory
    from quadsim.vehicles import get_vehicle
    traj = make_trajectory("lemniscate")
    ctrl = make_controller("mpc", get_vehicle("pae_x450"))
    sim = make_tracking_sim(traj, ctrl, "wind", seed=1)
    sim.set_trajectory(traj)
    L = sim.run(traj.duration)
    t = np.asarray(L["t"])
    p = np.stack([np.asarray(L[f"pos_{i}"]) for i in range(3)], 1)
    r = np.stack([np.asarray(L[f"ref_{i}"]) for i in range(3)], 1)
    err = np.linalg.norm(p - r, axis=1)
    rmse = float(np.sqrt(np.mean(err ** 2)))

    fig = plt.figure(figsize=(16.8 * CM, 6.6 * CM))
    ax = fig.add_axes([-0.02, -0.06, 0.62, 1.12], projection="3d")
    E, N, U = p[:, 1], p[:, 0], -p[:, 2]
    Er, Nr, Ur = r[:, 1], r[:, 0], -r[:, 2]
    ax.plot(Er, Nr, np.zeros_like(Ur), color=RULE, lw=1.0)
    ax.plot(Er, Nr, Ur, color=MAG, lw=1.0, ls=(0, (4, 2)), label="referencia")
    ax.plot(E, N, U, color=NAVY, lw=1.3, label="vuelo simulado")
    for k in np.linspace(0, len(E) - 1, 9).astype(int)[1:-1]:
        ax.plot([E[k], E[k]], [N[k], N[k]], [0, U[k]], color=RULE, lw=0.5)
    ax.scatter([E[-1]], [N[-1]], [U[-1]], color=CYAN, s=18, zorder=5)
    ax.set_xlabel("Este [m]", fontsize=7, labelpad=-6)
    ax.set_ylabel("Norte [m]", fontsize=7, labelpad=-4)
    ax.set_zlabel("Altura [m]", fontsize=7, labelpad=-6)
    ax.tick_params(labelsize=6, pad=-2)
    ax.set_zlim(0, 2.0)
    ax.view_init(elev=24, azim=-58)
    for a in (ax.xaxis, ax.yaxis, ax.zaxis):
        a.pane.set_facecolor((0.96, 0.97, 0.98, 1.0))
        a.pane.set_edgecolor(RULE)
        a._axinfo["grid"]["color"] = (0.85, 0.88, 0.91, 1)
        a._axinfo["grid"]["linewidth"] = 0.4
    ax.set_box_aspect((1.0, 2.6, 0.9))
    ax.legend(loc="upper left", fontsize=6.5, frameon=False, bbox_to_anchor=(0.1, 0.95))
    # error
    ax2 = fig.add_axes([0.66, 0.2, 0.32, 0.66])
    ax2.fill_between(t, err * 100, color=CYAN, alpha=0.25, lw=0)
    ax2.plot(t, err * 100, color=CYAN_D, lw=0.9)
    ax2.set_xlabel("Tiempo simulado [s]", fontsize=7)
    ax2.set_ylabel("Error de seguimiento [cm]", fontsize=7)
    ax2.tick_params(labelsize=6.5)
    for s in ("right", "top"):
        ax2.spines[s].set_visible(False)
    ax2.set_title(f"RMSE 3D = {rmse * 100:.2f} cm · viento 3 m/s + Dryden + ráfaga",
                  fontsize=7, color=NAVY, loc="left")
    save(fig, "portada_lemniscata.png")
    return rmse


# --------------------------------------------------------------- modelo (PDF -> PNG)
def figs_modelo():
    src = os.path.join(REPO, "docs", "modelo", "figuras")
    for pdf in sorted(glob.glob(os.path.join(src, "*.pdf"))):
        base = os.path.splitext(os.path.basename(pdf))[0]
        dst = os.path.join(OUT, "modelo_" + base)
        subprocess.run(["pdftoppm", "-png", "-r", "300", "-singlefile", pdf, dst], check=True)
        print("figura:", dst + ".png")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sin-sim", action="store_true")
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    fig_capas()
    fig_lazo()
    fig_relojes()
    fig_hilos()
    fig_flujo()
    figs_modelo()
    if not a.sin_sim:
        rmse = fig_portada()
        print(f"RMSE portada: {rmse * 100:.2f} cm")


if __name__ == "__main__":
    main()
