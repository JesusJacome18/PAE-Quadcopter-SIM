"""
docs/modelo/fig_marco.py
========================
Figura de convenciones: vista superior del bastidor en X (PX4 Quad X) con la
numeracion de motores, el sentido de giro de cada helice y los ejes FRD, mas
una vista lateral con NED, empuje y gravedad. La geometria se lee del propio
nucleo (quadsim.vehicles.make_rotors) para que la figura no pueda contradecir
al codigo.
"""

import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch, Arc

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from quadsim.vehicles import get_vehicle  # noqa: E402

C1, C2, C3, C7 = "#2a78d6", "#eb6834", "#1baf7a", "#4a3aa7"
INK, INK2, GRID = "#0b0b0b", "#52514e", "#dcdbd5"
plt.rcParams.update({"font.size": 9, "font.family": "DejaVu Sans",
                     "mathtext.fontset": "dejavusans"})


def spin_arrow(ax, cx, cy, r, ccw, color):
    """Flecha circular. ccw=True: antihorario visto desde ARRIBA (la vista)."""
    if ccw:
        a0, a1 = 200, 520
    else:
        a0, a1 = 520, 200
    th = [math.radians(a0 + (a1 - a0) * k / 40) for k in range(41)]
    xs = [cx + r * math.cos(t) for t in th]
    ys = [cy + r * math.sin(t) for t in th]
    ax.plot(xs[:-3], ys[:-3], color=color, lw=1.6)
    ax.annotate("", xy=(xs[-1], ys[-1]), xytext=(xs[-4], ys[-4]),
                arrowprops=dict(arrowstyle="-|>", color=color, lw=1.6,
                                mutation_scale=11))


def main():
    p = get_vehicle("pae_x450")
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(7.4, 3.9),
                                 gridspec_kw={"width_ratios": [1.25, 1]})
    # ------------------------------------------------------------- vista superior
    # coordenadas de pagina: horizontal = y_B (derecha), vertical = x_B (adelante)
    ax.set_aspect("equal")
    ax.axis("off")
    R = 0.115   # radio de helice dibujado (~9.4 in / 2)
    for r in p.rotors:
        X, Y = r.y, r.x
        ax.plot([0, X], [0, Y], color=INK2, lw=4, solid_capstyle="round", zorder=1)
    ax.add_patch(Circle((0, 0), 0.045, color="#cfcdc4", zorder=2))
    for r in p.rotors:
        X, Y = r.y, r.x
        ccw = r.spin > 0
        col = C1 if ccw else C2
        ax.add_patch(Circle((X, Y), R, facecolor="none", edgecolor=col, lw=1.0,
                            ls=(0, (3, 2)), zorder=2))
        ax.add_patch(Circle((X, Y), 0.022, color=col, zorder=3))
        spin_arrow(ax, X, Y, 0.072, ccw, col)
        num, pos = r.label.split()
        ox = 0.06 if X > 0 else -0.06
        ax.text(X + ox * 2.35, Y + (0.11 if Y > 0 else -0.11),
                f"{num} ({pos})\n{'CCW' if ccw else 'CW'}, $e_{num[1]}={r.spin:+d}$",
                ha="center", va="center", fontsize=8, color=INK)
    # ejes del cuerpo
    ax.annotate("", xy=(0, 0.17), xytext=(0, 0),
                arrowprops=dict(arrowstyle="-|>", color=INK, lw=1.4))
    ax.annotate("", xy=(0.17, 0), xytext=(0, 0),
                arrowprops=dict(arrowstyle="-|>", color=INK, lw=1.4))
    ax.text(0.0, 0.178, "$x_B$", fontsize=9, va="bottom", ha="center")
    ax.text(0.178, 0.0, "$y_B$", fontsize=9, va="center")
    ax.add_patch(Circle((0, 0), 0.016, facecolor="white", edgecolor=INK, lw=1, zorder=4))
    ax.plot([-0.011, 0.011], [-0.011, 0.011], color=INK, lw=0.9, zorder=5)
    ax.plot([-0.011, 0.011], [0.011, -0.011], color=INK, lw=0.9, zorder=5)
    ax.text(-0.03, 0.0, "$z_B$ (abajo)", fontsize=7.5,
            ha="right", va="center", color=INK2)
    # yaw positivo: horario visto desde arriba
    arc = Arc((0, 0), 0.62, 0.62, angle=0, theta1=60, theta2=120, color=C7, lw=1.4)
    ax.add_patch(arc)
    ax.annotate("", xy=(0.31 * math.cos(math.radians(58)), 0.31 * math.sin(math.radians(58))),
                xytext=(0.31 * math.cos(math.radians(64)), 0.31 * math.sin(math.radians(64))),
                arrowprops=dict(arrowstyle="-|>", color=C7, lw=1.4, mutation_scale=11))
    ax.text(0, 0.335, "$\\psi>0$: morro a la derecha (rumbo)", ha="center",
            fontsize=8, color=C7)
    a = p.arm / math.sqrt(2)
    ax.text(0, -0.36, f"brazo $\\ell$ = {p.arm} m;  $|x_i| = |y_i| = \\ell/\\sqrt{{2}}$ = {a:.4f} m",
            ha="center", fontsize=8, color=INK2)
    ax.set_xlim(-0.36, 0.36)
    ax.set_ylim(-0.39, 0.37)
    ax.set_title("(a) Vista superior: PX4 Quad X, marco cuerpo FRD", fontsize=9)

    # ------------------------------------------------------------- vista lateral
    bx.set_aspect("equal")
    bx.axis("off")
    # NED: x norte a la derecha, z abajo
    o = (-0.34, -0.06)
    bx.annotate("", xy=(o[0] + 0.18, o[1]), xytext=o,
                arrowprops=dict(arrowstyle="-|>", color=INK, lw=1.3))
    bx.annotate("", xy=(o[0], o[1] - 0.15), xytext=o,
                arrowprops=dict(arrowstyle="-|>", color=INK, lw=1.3))
    bx.text(o[0] + 0.19, o[1], "$x_E$ (Norte)", va="center", fontsize=8)
    bx.text(o[0] + 0.015, o[1] - 0.15, "$z_E$ (abajo)", fontsize=8, va="center")
    bx.text(o[0] - 0.02, o[1] + 0.02, "{E}: NED", fontsize=8, ha="left", va="bottom", color=INK2)
    # dron inclinado con pitch negativo (morro abajo) volando al norte
    th = math.radians(-15)
    cx, cy = 0.14, 0.04
    L = 0.17
    def body(xb, zb):
        # x_B adelante, z_B abajo, rotado por pitch th en el plano x-z
        X = xb * math.cos(th) + zb * math.sin(th)
        Z = -xb * math.sin(th) + zb * math.cos(th)
        return cx + X, cy - Z
    p1, p2 = body(-L, 0), body(L, 0)
    bx.plot([p1[0], p2[0]], [p1[1], p2[1]], color=INK2, lw=4, solid_capstyle="round")
    for s in (-L, L):
        q1, q2 = body(s - 0.06, -0.025), body(s + 0.06, -0.025)
        bx.plot([q1[0], q2[0]], [q1[1], q2[1]], color=C1, lw=2)
    t0 = body(0, 0)
    t1 = body(0, -0.18)
    bx.annotate("", xy=t1, xytext=t0, arrowprops=dict(arrowstyle="-|>", color=C3, lw=1.6))
    bx.text(t1[0] - 0.01, t1[1] + 0.005, "$T = \\sum T_i$\n(a lo largo de $-z_B$)",
            fontsize=8, color=INK, ha="right", va="bottom")
    g1 = (cx, cy - 0.165)
    bx.annotate("", xy=g1, xytext=t0, arrowprops=dict(arrowstyle="-|>", color=C8 if False else "#e34948", lw=1.6))
    bx.text(g1[0] + 0.01, g1[1], "$m g\\, e_3$", fontsize=8, va="top")
    xb1 = body(0.15, 0)
    bx.annotate("", xy=body(0.27, 0), xytext=xb1, arrowprops=dict(arrowstyle="-|>", color=INK, lw=1.1))
    bx.text(*body(0.28, 0.02), "$x_B$", fontsize=8, va="top")
    bx.text(0.04, -0.37, "$\\theta<0$ (morro abajo) $\\Rightarrow$ acelera hacia $+x_E$",
            ha="center", fontsize=8, color=INK2)
    bx.set_xlim(-0.36, 0.42)
    bx.set_ylim(-0.40, 0.36)
    bx.set_title("(b) Vista lateral: empuje y gravedad", fontsize=9)
    fig.tight_layout()
    out = Path(__file__).resolve().parent / "figuras"
    out.mkdir(exist_ok=True)
    fig.savefig(out / "marco_quad_x.pdf", bbox_inches="tight")
    fig.savefig(out / "marco_quad_x.png", dpi=170, bbox_inches="tight")


if __name__ == "__main__":
    main()
