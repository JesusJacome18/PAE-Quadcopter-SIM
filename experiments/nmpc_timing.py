"""
experiments/nmpc_timing.py
==========================
Tiempo de computo por llamada del NMPC (y del MPC lineal, como referencia)
en lazo cerrado sobre el circulo de las pruebas (radio 1.5 m, periodo 8 s,
1 vuelta). El tiempo de reloj depende de la CPU, del interprete y de la
carga de la maquina, por eso se guarda junto con el entorno de la medicion.

Uso:
    PYTHONPATH=. python experiments/nmpc_timing.py [--out experiments/results/nmpc_timing.json]
"""

from __future__ import annotations

import argparse
import json
import os
import platform
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from quadsim.control.position import make_controller   # noqa: E402
from quadsim.scenarios import make_tracking_sim          # noqa: E402
from quadsim.trajectories import make_trajectory         # noqa: E402


def measure(key: str) -> dict:
    traj = make_trajectory("circle", radius=1.5, period=8.0, laps=1, t_ramp=2.0)
    ctrl = make_controller(key, "pae_x450")
    sim = make_tracking_sim(traj, ctrl, "nominal")
    ms = []
    for _ in range(int(round(traj.duration / sim.dt_ctrl))):
        sim.step_control()
        if sim.k % sim.outer_every == 1:
            ms.append(ctrl.stats["solve_ms"])
    ms = np.array(ms)
    return {"llamadas": int(len(ms)), "mediana_ms": float(np.median(ms)),
            "p95_ms": float(np.percentile(ms, 95)), "p99_ms": float(np.percentile(ms, 99)),
            "max_ms": float(ms.max()),
            "lm_fallos": int(getattr(ctrl, "lm_failures", 0))}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "experiments" / "results" / "nmpc_timing.json"))
    a = ap.parse_args(argv)
    res = {"entorno": {"plataforma": platform.platform(),
                       "procesador": platform.processor() or platform.machine(),
                       "cpu": os.cpu_count(), "python": sys.version.split()[0],
                       "numpy": np.__version__, "fecha": time.strftime("%Y-%m-%d")}}
    for key in ("mpc", "nmpc"):
        res[key] = measure(key)
        print(key, res[key], flush=True)
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps(res, indent=1), encoding="utf-8")
    print("guardado en", a.out)


if __name__ == "__main__":
    main()
