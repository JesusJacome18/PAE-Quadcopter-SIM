"""
experiments/delay_margins.py
============================
Efecto del retardo de computo y de la tasa del ESC sobre la estabilidad del
lazo de tasa de roll del PAE-X450, para documentar cuanto margen se pierde
con cada eslabon de la cadena de actuacion que el simulador puede modelar
(SimConfig.compute_delay_samples, esc_rate_hz, esc_pulse_delay_s).

Dos estimaciones independientes:
  * dominio de la frecuencia (quadsim.control.margins): margen de fase, de
    ganancia y multiplicador maximo de ganancia (Kp, Ki, Kd juntos) del lazo
    lineal en hover;
  * dominio del tiempo: simulacion 6-DOF completa con un escalon de roll de
    10 grados y ganancias de tasa multiplicadas por m; biseccion sobre m
    hasta que aparece una oscilacion sostenida (desviacion estandar de la
    tasa de roll verdadera > 3 deg/s en el ultimo segundo; estable da
    ~0.4 deg/s y el ciclo limite, acotado por la saturacion, 12-24 deg/s) o
    el dron se inclina > 60 grados.

Las ganancias son las SUGERIDAS para la configuracion por defecto (las mismas
en todos los casos: se mide que pasa si se vuela esa sintonia con otra
cadena de actuacion, que es el riesgo al transferir la sintonia al dron
real). Los valores reales del ESC y del firmware estan PENDIENTES de
identificar; las filas son escenarios, no mediciones del dron.

Uso:
    PYTHONPATH=. python experiments/delay_margins.py [--no-time] [--out DIR]
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from quadsim.control.margins import sim_rate_loop_margins   # noqa: E402
from quadsim.dynamics import IQ, IW                           # noqa: E402
from quadsim import math3d as m3                              # noqa: E402
from quadsim.simulator import SimConfig, Simulation           # noqa: E402

CASES = [
    ("continuo, sin retardo de computo (por defecto)", {}),
    ("+1 muestra de computo (2 ms)", {"compute_delay_samples": 1}),
    ("+2 muestras de computo (4 ms)", {"compute_delay_samples": 2}),
    ("ESC 400 Hz + 1 muestra", {"esc_rate_hz": 400.0, "compute_delay_samples": 1}),
    ("ESC 400 Hz + 1 muestra + pulso 1.5 ms",
     {"esc_rate_hz": 400.0, "compute_delay_samples": 1, "esc_pulse_delay_s": 0.0015}),
    ("ESC 50 Hz (PWM estandar) + pulso 1.5 ms",
     {"esc_rate_hz": 50.0, "esc_pulse_delay_s": 0.0015}),
]


def _sim(kw: dict) -> Simulation:
    sim = Simulation(SimConfig(mode="attitude", start_airborne=True, start_alt=2.0,
                               seed=1, **kw))
    # misma sintonia en todos los casos: la sugerida para la cadena por defecto
    ref = Simulation(SimConfig(mode="attitude", start_airborne=True, start_alt=2.0))
    for ax, g in ref.att_ctrl.get_gains().items():
        sim.att_ctrl.set_gains(ax, g["angle_kp"], g["rate_kp"], g["rate_ki"], g["rate_kd"])
    sim.set_attitude_setpoint(alt_m=2.0)
    return sim


def unstable(kw: dict, factor: float, T: float = 3.0) -> bool:
    sim = _sim(kw)
    for ax in ("roll", "pitch"):
        pid = sim.att_ctrl.rate_pid[ax]
        pid.g.kp *= factor
        pid.g.ki *= factor
        pid.g.kd *= factor
    w, n = [], int(round(T / sim.dt_ctrl))
    for k in range(n):
        if abs(sim.t - 0.5) < 1e-9:
            sim.set_attitude_setpoint(roll_deg=10.0)
        sim.step_control()
        r, p, _ = m3.q_to_euler(sim.x[IQ:IQ + 4])
        if not all(math.isfinite(v) for v in (r, p)) or max(abs(r), abs(p)) > math.radians(60):
            return True
        if sim.t > T - 1.0:
            w.append(sim.x[IW])
    mu = sum(w) / len(w)
    sd = math.sqrt(sum((v - mu) ** 2 for v in w) / len(w))
    return math.degrees(sd) > 3.0


def max_stable_factor(kw: dict, lo: float = 1.0, hi: float = 8.0, iters: int = 6) -> float:
    if unstable(kw, lo):
        return lo
    if not unstable(kw, hi):
        return hi
    for _ in range(iters):
        mid = math.sqrt(lo * hi)
        if unstable(kw, mid):
            hi = mid
        else:
            lo = mid
    return lo


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-time", action="store_true", help="solo el analisis en frecuencia")
    ap.add_argument("--out", default=str(ROOT / "experiments" / "results" / "delay_margins"))
    a = ap.parse_args(argv)
    rows = []
    for label, kw in CASES:
        sim = _sim(kw)
        m = sim_rate_loop_margins(sim, "roll")
        row = {"caso": label, "config": kw, **m}
        if not a.no_time:
            row["max_gain_factor_time"] = max_stable_factor(kw)
        rows.append(row)
        print(f"{label:45s} cruce {m['f_cross_hz']:.2f} Hz  MF {m['phase_margin_deg']:.1f} deg  "
              f"MG {m['gain_margin_db']:.1f} dB  m_max(frec) {m['max_gain_factor']:.2f}"
              + (f"  m_max(tiempo) {row['max_gain_factor_time']:.2f}" if not a.no_time else ""),
              flush=True)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "delay_margins.json").write_text(json.dumps(rows, indent=1, ensure_ascii=False),
                                            encoding="utf-8")
    md = ["| caso | cruce [Hz] | margen de fase [°] | margen de ganancia [dB] | "
          "ganancia máx. (frecuencia) | ganancia máx. (simulación) |", "|---|---|---|---|---|---|"]
    for r in rows:
        md.append(f"| {r['caso']} | {r['f_cross_hz']:.2f} | {r['phase_margin_deg']:.1f} | "
                  f"{r['gain_margin_db']:.1f} | {r['max_gain_factor']:.2f}× | "
                  + (f"{r['max_gain_factor_time']:.2f}× |" if 'max_gain_factor_time' in r else "— |"))
    (out / "delay_margins.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    print(f"resultados en {out}")
    return rows


if __name__ == "__main__":
    main()
