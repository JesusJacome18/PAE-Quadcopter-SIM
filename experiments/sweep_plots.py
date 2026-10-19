"""
Graficas de los barridos de sensibilidad (matplotlib) para la linea de
comandos: experiments/sweep_cli.py las llama al terminar un barrido. La
interfaz grafica dibuja sus propias graficas con pyqtgraph; este modulo no
forma parte del programa.
"""

from __future__ import annotations

import math
import os
import sys
from typing import Dict, List, Optional

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from quadsim.objectives import SCENARIOS  # noqa: E402
from quadsim.sweeps import SweepResult, spearman  # noqa: E402

# Paleta categorica de referencia (orden fijo, validada para daltonismo en
# pares adyacentes) y rampa secuencial de un solo tono (azul) para mapas.
_CAT = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300",
        "#4a3aa7", "#e34948"]
_SEQ = ["#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95", "#0d366b"]


def _mpl():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import LinearSegmentedColormap
    plt.rcParams.update({
        "figure.dpi": 110, "axes.spines.top": False, "axes.spines.right": False,
        "axes.grid": True, "grid.color": "#e4e3df", "grid.linewidth": 0.6,
        "axes.edgecolor": "#8a8984", "axes.labelcolor": "#2b2b29",
        "xtick.color": "#52514e", "ytick.color": "#52514e", "font.size": 9,
        "axes.titlesize": 10, "legend.frameon": False, "lines.linewidth": 2.0})
    cmap = LinearSegmentedColormap.from_list("azul", _SEQ)
    return plt, cmap


def plot_result(res: SweepResult, out_dir: Optional[str] = None,
                metrics: Optional[List[str]] = None) -> List[str]:
    """Genera PNGs: curvas (1 parametro numerico), mapas de calor (2),
    sensibilidad (>2 o diseno aleatorio) y Pareto. Devuelve las rutas."""
    out_dir = out_dir or res.out_dir
    if out_dir is None:
        return []
    plt, cmap = _mpl()
    sp = res.spec
    rows = [r for r in res.rows if not r["is_baseline"] and r.get("phase") == "diseno"]
    if not rows:
        return []
    metrics = metrics or (["cost"] + list(sp.cost_weights)
                          + [m for m in (sp.report_metrics or SCENARIOS[sp.scenario].KEY_METRICS)
                             if m not in sp.cost_weights][:3])
    # las condiciones nunca son ejes: agrupan curvas o separan mapas
    num = [p for p in sp.params if p.numeric and len(p.grid_values()) > 1
           and p.path not in sp.conditions]
    cat = [p for p in sp.params if p not in num]
    files = []
    base = res.baseline

    def label(p):
        return p.label or p.col

    if len(num) == 1 and sp.design == "grid":
        x = num[0]
        groups: Dict[str, list] = {}
        gcat = [c for c in cat if len(c.grid_values()) > 1]   # los de un solo valor no agrupan
        for r in rows:
            if len(gcat) == 1 and gcat[0].kind == "choice" and gcat[0].value_labels:
                key = str(r[gcat[0].col])
            else:
                key = " · ".join(f"{c.label or c.col.split('.')[-1]}={r[c.col]}"
                                 for c in gcat) if gcat else "—"
            groups.setdefault(key, []).append(r)
        nm = len(metrics)
        ncol = min(3, nm)
        nrow = int(math.ceil(nm / ncol))
        fig, axs = plt.subplots(nrow, ncol, figsize=(4.2 * ncol, 3.1 * nrow), squeeze=False)
        for k, m in enumerate(metrics):
            ax = axs[k // ncol][k % ncol]
            for gi, (g, rr) in enumerate(list(groups.items())[:8]):
                rr = sorted(rr, key=lambda r: r["params"][x.path])
                xs = [r["params"][x.path] for r in rr]
                ys = [r.get(m, np.nan) for r in rr]
                ys = [y if np.isfinite(y) else np.nan for y in ys]
                ax.plot(xs, ys, "-o", ms=4, color=_CAT[gi], label=g)
            if sp.conditions:
                if m == "cost" and res.robust:
                    rb = sorted([r for r in res.robust if r.get("phase") == "diseno"],
                                key=lambda r: r["params"][x.path])
                    ax.plot([r["params"][x.path] for r in rb], [r["cost"] for r in rb], "-",
                            color="#2b2b29", lw=2.5, label=f"robusto ({sp.robust_stat})")
            elif base is not None and np.isfinite(base.get(m, np.nan)):
                ax.axhline(base[m], color="#8a8984", lw=1, ls="--", label="defecto")
            if x.log:
                ax.set_xscale("log")
            ax.set_xlabel(label(x))
            ax.set_title(m)
        for k in range(nm, nrow * ncol):
            axs[k // ncol][k % ncol].axis("off")
        h, lab = [], []
        for axrow in axs:                    # leyenda unica con todas las series
            for ax_ in axrow:
                for hh, ll in zip(*ax_.get_legend_handles_labels()):
                    if ll not in lab:
                        h.append(hh)
                        lab.append(ll)
        fig.legend(h, lab, loc="upper center", ncol=min(len(lab), 6),
                   bbox_to_anchor=(0.5, 1.02))
        fig.suptitle(f"{sp.label or sp.name}", y=1.07)
        fig.tight_layout()
        f = os.path.join(out_dir, "curvas.png")
        fig.savefig(f, bbox_inches="tight")
        plt.close(fig)
        files.append(f)

    if len(num) == 2 and sp.design == "grid":
        px, py = num
        b = res.best()
        tcat = [c for c in cat if c.path not in sp.conditions]
        fixed_t = {c.col: b[c.col] for c in tcat} if b else {}
        slices = []          # (sufijo de archivo, filtro de columnas, filas)
        if sp.conditions:
            for cc in sp.condition_combos():
                fx = dict(fixed_t)
                fx.update({p.col: p.label_of(cc[p.path]) for p in sp.cond_params})
                slices.append(("_" + "_".join(f"{p.col.split('.')[-1]}-{fx[p.col]}"
                                               for p in sp.cond_params), fx, rows))
            slices.append(("_robusto", dict(fixed_t),
                           [r for r in res.robust if r.get("phase") == "diseno"]))
        else:
            slices.append(("", fixed_t, rows))
        xs = px.grid_values()
        ys = py.grid_values()
        for suffix, fixed_cat, pool in slices:
            rr = [r for r in pool if all(r.get(c) == v for c, v in fixed_cat.items())]
            for m in metrics:
                Z = np.full((len(ys), len(xs)), np.nan)
                for r in rr:
                    try:
                        i = ys.index(r["params"][py.path])
                        j = xs.index(r["params"][px.path])
                    except ValueError:
                        continue
                    v = r.get(m, np.nan)
                    Z[i, j] = v if np.isfinite(v) else np.nan
                if not np.isfinite(Z).any():
                    continue
                fig, ax = plt.subplots(figsize=(5.6, 4.2))
                fz = Z[np.isfinite(Z)]
                norm = None
                if fz.size and fz.min() > 0 and fz.max() / fz.min() > 20:
                    from matplotlib.colors import LogNorm   # rango enorme: escala log
                    norm = LogNorm(vmin=fz.min(), vmax=fz.max())
                im = ax.imshow(Z, origin="lower", aspect="auto", cmap=cmap, norm=norm)
                ax.set_xticks(range(len(xs)))
                ax.set_xticklabels([px.label_of(v) for v in xs], rotation=45)
                ax.set_yticks(range(len(ys)))
                ax.set_yticklabels([py.label_of(v) for v in ys])
                ax.set_xlabel(label(px))
                ax.set_ylabel(label(py))
                ax.grid(False)
                # valor en cada celda (texto en tinta, no en color de serie)
                finite = Z[np.isfinite(Z)]
                mid = (np.exp(np.nanmedian(np.log(finite))) if norm is not None
                       else np.nanmedian(finite)) if finite.size else 0
                for i in range(len(ys)):
                    for j in range(len(xs)):
                        if np.isfinite(Z[i, j]):
                            ax.text(j, i, f"{Z[i, j]:.3g}", ha="center", va="center",
                                    fontsize=6.5, color="#ffffff" if Z[i, j] > mid else "#0b0b0b")
                        else:
                            ax.text(j, i, "×", ha="center", va="center", fontsize=8,
                                    color="#e34948")
                if b is not None and m == "cost":
                    try:
                        ax.plot(xs.index(b["params"][px.path]), ys.index(b["params"][py.path]),
                                "s", ms=14, mfc="none", mec="#eb6834", mew=2)
                    except ValueError:
                        pass
                fig.colorbar(im, ax=ax, label=m)
                sub = ", ".join(f"{k}={v}" for k, v in fixed_cat.items())
                if suffix == "_robusto":
                    sub = (sub + ", " if sub else "") + f"robusto ({sp.robust_stat})"
                ax.set_title(f"{sp.label or sp.name}: {m}" + (f"\n{sub}" if sub else ""))
                fig.tight_layout()
                f = os.path.join(out_dir, f"mapa_{m}{suffix}.png")
                fig.savefig(f)
                plt.close(fig)
                files.append(f)

    if len(num) > 2 or (num and sp.design != "grid"):
        n = len(num)
        ncol = min(3, n)
        nrow = int(math.ceil(n / ncol))
        fig, axs = plt.subplots(nrow, ncol, figsize=(4.0 * ncol, 3.0 * nrow), squeeze=False)
        allr = [r for r in res.rows if not r["is_baseline"] and np.isfinite(r["cost"])]
        for k, p in enumerate(num):
            ax = axs[k // ncol][k % ncol]
            xs = [r["params"].get(p.path) for r in allr]
            ys = [r["cost"] for r in allr]
            ax.plot(xs, ys, "o", ms=4, color=_CAT[0], alpha=0.6)
            if p.log:
                ax.set_xscale("log")
            rho = spearman(xs, ys)
            ax.set_title(f"{label(p)}  (ρ={rho:.2f})")
            ax.set_ylabel("J")
        for k in range(n, nrow * ncol):
            axs[k // ncol][k % ncol].axis("off")
        fig.tight_layout()
        f = os.path.join(out_dir, "sensibilidad.png")
        fig.savefig(f)
        plt.close(fig)
        files.append(f)

    if sp.pareto:
        o1, o2 = sp.pareto
        allr = [r for r in res.rows if not r["is_baseline"]
                and np.isfinite(r.get(o1, np.nan)) and np.isfinite(r.get(o2, np.nan))
                and r.get("crash_rate", 0) == 0]
        if allr:
            fig, ax = plt.subplots(figsize=(5.6, 4.2))
            ax.plot([r[o1] for r in allr], [r[o2] for r in allr], "o", ms=5,
                    color="#b7d3f6", label="puntos")
            fr = res.pareto
            ax.plot([r[o1] for r in fr], [r[o2] for r in fr], "-o", ms=6, color=_CAT[0],
                    label="frente de Pareto")
            step = max(1, len(fr) // 8)          # etiquetas selectivas, no todas
            for r in fr[::step]:
                ax.annotate(" " + ", ".join(str(r[p.col]) for p in sp.params),
                            (r[o1], r[o2]), fontsize=6, color="#52514e")
            for o, setter in ((o1, ax.set_xscale), (o2, ax.set_yscale)):
                v = np.array([r[o] for r in allr])
                if v.min() > 0 and v.max() / v.min() > 20:
                    setter("log")
            if base is not None and np.isfinite(base.get(o1, np.nan)):
                ax.plot(base[o1], base[o2], "D", ms=8, color="#2b2b29", label="defecto")
            b = res.best()
            if b is not None:
                ax.plot(b[o1], b[o2], "s", ms=10, mfc="none", mec=_CAT[1], mew=2,
                        label="mejor J")
            ax.set_xlabel(o1)
            ax.set_ylabel(o2)
            ax.legend()
            ax.set_title(f"{sp.label or sp.name}: Pareto")
            fig.tight_layout()
            f = os.path.join(out_dir, "pareto.png")
            fig.savefig(f)
            plt.close(fig)
            files.append(f)
    return files
