"""
Verificacion del dataset generado por experiments/generar_dataset.py, con
figuras de cobertura.

    python experiments/verificar_dataset.py experiments/results/dataset_pae_x450

Las revisiones numericas estan en quadsim/dataset_verificacion.py; aqui se
agregan las figuras (figuras/cobertura.png, recorridos.png, vuelo_ejemplo.png).
"""

from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from quadsim.dataset_verificacion import verificar as _verificar  # noqa: E402

AZUL, NARANJA, AQUA, GRIS_TXT, GRIS_EJE = "#2a78d6", "#eb6834", "#1baf7a", "#52514e", "#c3c2b7"


def verificar(carpeta: str) -> bool:
    return _verificar(carpeta, figuras=_figuras)


def _figuras(carpeta, vuelos, tilt, vel, pwm, rates):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    os.makedirs(os.path.join(carpeta, "figuras"), exist_ok=True)
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": GRIS_EJE, "axes.labelcolor": GRIS_TXT,
                         "xtick.color": GRIS_TXT, "ytick.color": GRIS_TXT,
                         "axes.spines.top": False, "axes.spines.right": False,
                         "axes.grid": True, "grid.color": "#e8e7e3", "grid.linewidth": 0.6})

    # cobertura: cuatro histogramas (una sola serie cada uno)
    fig, ax = plt.subplots(2, 2, figsize=(9, 6))
    for a, datos, tit, uni in ((ax[0, 0], tilt, "Inclinación", "°"),
                               (ax[0, 1], vel, "Rapidez", "m/s"),
                               (ax[1, 0], pwm, "PWM de los motores", "µs"),
                               (ax[1, 1], np.degrees(rates), "Velocidad angular (p, q, r)", "°/s")):
        a.hist(datos, bins=80, color=AZUL, edgecolor="white", linewidth=0.3)
        a.set_title(tit, loc="left", fontsize=10, color="#0b0b0b")
        a.set_xlabel(uni)
        a.set_yscale("log")
        a.set_ylabel("muestras (escala log)")
    fig.suptitle("Cobertura del dataset: todas las muestras de todos los vuelos", x=0.01,
                 ha="left", fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(carpeta, "figuras", "cobertura.png"), dpi=130)
    plt.close(fig)

    # vista superior de todos los recorridos
    fig, ax = plt.subplots(figsize=(6.5, 6))
    for v, d in vuelos:
        ax.plot(d["pos_e"], d["pos_n"], color=AZUL, lw=0.6, alpha=0.35)
    ax.set_aspect("equal")
    ax.set_xlabel("Este [m]")
    ax.set_ylabel("Norte [m]")
    ax.set_title("Recorridos de todos los vuelos (vista superior)", loc="left", fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(carpeta, "figuras", "recorridos.png"), dpi=130)
    plt.close(fig)

    # un vuelo de ejemplo: entradas, actitud y sensor contra verdad
    ej = next(((v, d) for v, d in vuelos if v["tipo"] == "figura_8"), vuelos[0])
    v, d = ej
    m = d["t"] <= min(d["t"][-1], 20.0)
    fig, ax = plt.subplots(3, 1, figsize=(9, 7.5), sharex=True)
    cols4 = [AZUL, NARANJA, AQUA, "#eda100"]
    for i in range(4):
        ax[0].plot(d["t"][m], d[f"pwm_{i + 1}"][m], lw=0.8, color=cols4[i], label=f"motor {i + 1}")
    ax[0].set_ylabel("PWM [µs]")
    ax[0].legend(ncol=4, frameon=False, loc="upper right", fontsize=8)
    ax[0].set_title(f"Vuelo {v['id']:03d} ({v['tipo']}): primeros 20 s", loc="left", fontsize=10)
    ax[1].plot(d["t"][m], np.degrees(d["roll"][m]), lw=1.2, color=AZUL, label="alabeo")
    ax[1].plot(d["t"][m], np.degrees(d["pitch"][m]), lw=1.2, color=NARANJA, label="cabeceo")
    ax[1].set_ylabel("ángulo [°]")
    ax[1].legend(ncol=2, frameon=False, loc="upper right", fontsize=8)
    ax[2].plot(d["t"][m], np.degrees(d["gyro_x"][m]), lw=0.6, color=GRIS_EJE, label="giroscopio x")
    ax[2].plot(d["t"][m], np.degrees(d["p"][m]), lw=1.2, color=AZUL, label="p real")
    ax[2].set_ylabel("vel. angular [°/s]")
    ax[2].set_xlabel("t [s]")
    ax[2].legend(ncol=2, frameon=False, loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(carpeta, "figuras", "vuelo_ejemplo.png"), dpi=130)
    plt.close(fig)


if __name__ == "__main__":
    ok = verificar(sys.argv[1] if len(sys.argv) > 1 else
                   os.path.join(os.path.dirname(os.path.abspath(__file__)), "results",
                                "dataset_pae_x450"))
    sys.exit(0 if ok else 1)
