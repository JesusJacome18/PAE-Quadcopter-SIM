"""
sim2real.replay
===============
Dos comparaciones con un registro de ArduCopter:

1. `replay_rate_pid`: alimenta el PID de tasa de QuadSim
   (quadsim/control/ardupilot.py) con la tasa objetivo y la tasa medida del
   registro (RATE.RDes, RATE.R) y compara su salida con la del firmware
   (RATE.ROut). No depende del modelo del dron: verifica que el PID de
   QuadSim calcula lo mismo que ArduPilot con los mismos parametros.

2. `simulate_log`: repite el ensayo en QuadSim. Toma del registro los
   parametros (PARM), la actitud inicial (ANG), las palancas del piloto
   (RCIN -> angulos, como Mode::get_pilot_desired_lean_angles_rad) y el
   acelerador del piloto (CTUN.ThI), y compara la actitud, las tasas y las
   salidas del simulador con las del dron. Aqui si entra el modelo fisico
   (inercia, motores, banco): las diferencias son lo que hay que corregir.

ORDEN DEL LAZO: en cada ciclo ArduCopter corre primero el PID de tasa con el
objetivo calculado en el ciclo anterior y despues el controlador de actitud
calcula el objetivo nuevo, que es el que queda en RATE.RDes. Por eso la
salida ROut[k] sale de RDes[k-1] y R[k] (`lag = 1`).
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from quadsim.bench import BenchConfig
from quadsim.control.ardupilot import ArduPID, ArduPilotParams
from quadsim.simulator import FilterStage, SimConfig, Simulation

from .binlog import BinLog

AXES = ("roll", "pitch", "yaw")
_RATE_FIELDS = {"roll": ("RDes", "R", "ROut"), "pitch": ("PDes", "P", "POut"),
                "yaw": ("YDes", "Y", "YOut")}
_PID_MSG = {"roll": "PIDR", "pitch": "PIDP", "yaw": "PIDY"}


def params_from_log(log: BinLog) -> ArduPilotParams:
    if not log.params:
        raise ValueError(f"{log.name} no trae parametros (mensajes PARM)")
    return ArduPilotParams.from_dict(log.params, source=log.name)


# ---------------------------------------------------------------------------
# 1) PID de tasa
# ---------------------------------------------------------------------------
@dataclass
class PIDReplay:
    axis: str
    t: np.ndarray
    real: np.ndarray
    model: np.ndarray
    rms: float
    max_abs: float
    rate_hz: float
    warnings: List[str] = field(default_factory=list)


def replay_rate_pid(log: BinLog, axis: str = "roll", t0: float = None, t1: float = None,
                    lag: int = 1, prm: ArduPilotParams = None) -> PIDReplay:
    if not log.has("RATE"):
        raise ValueError("el registro no tiene mensajes RATE")
    prm = prm or params_from_log(log)
    fdes, fmeas, fout = _RATE_FIELDS[axis]
    t = log.t("RATE")
    sel = np.ones_like(t, dtype=bool)
    if t0 is not None:
        sel &= t >= t0
    if t1 is not None:
        sel &= t <= t1
    idx = np.nonzero(sel)[0]
    if len(idx) < lag + 3:
        raise ValueError("muy pocas muestras de RATE en el intervalo")
    des = np.radians(log.get("RATE", fdes))
    meas = np.radians(log.get("RATE", fmeas))
    out = log.get("RATE", fout)
    warns = []
    hz = log.rate_hz("RATE")
    if hz < 0.5 * prm.loop_rate_hz:
        warns.append(f"RATE esta a {hz:.0f} Hz y el lazo a {prm.loop_rate_hz:g} Hz: falta "
                     f"ATTITUDE_FAST en LOG_BITMASK; la repeticion del PID no es valida.")
    # banderas de saturacion e integrador inicial del mensaje PID, si existe
    pmsg = _PID_MSG[axis]
    have_pid = log.has(pmsg)
    pid = ArduPID(prm.rate[axis])
    first = idx[0] + lag
    if have_pid:
        # estado de los filtros e integrador del firmware justo antes del tramo
        tp = [t[first - 1]]
        pid.integrator = float(log.at(pmsg, "I", tp)[0])
        pid.target = float(log.at(pmsg, "Tar", tp)[0])
        pid.error = float(log.at(pmsg, "Err", tp)[0])
        kd = prm.rate[axis].d
        pid.derivative = float(log.at(pmsg, "D", tp)[0]) / kd if kd else 0.0
        pid._reset_filter = False
        flags = log.at(pmsg, "Flags", t).astype(int)
    else:
        flags = np.zeros_like(t, dtype=int)
    model = np.full(len(idx), np.nan)
    for j, k in enumerate(idx):
        if k < first:
            continue
        dt = t[k] - t[k - 1]
        if not dt > 0:
            dt = 1.0 / prm.loop_rate_hz
        # en ArduPilot el filtro del PID se reinicia en la primera llamada
        # la bandera LIMIT del mensaje PID es la que recibio update_all en ese ciclo
        u = pid.update_all(des[k - lag], meas[k], dt, bool(flags[k] & 1))
        model[j] = u + pid.get_ff()
    ok = np.isfinite(model)
    err = model[ok] - out[idx][ok]
    # se descartan las primeras muestras (filtros arrancando)
    skip = min(len(err) // 10, int(0.05 * max(hz, 1.0)))
    e = err[skip:] if len(err) > skip + 3 else err
    return PIDReplay(axis, t[idx], out[idx], model, float(np.sqrt(np.mean(e ** 2))),
                     float(np.max(np.abs(e))), hz, warns)


# ---------------------------------------------------------------------------
# 2) repeticion del ensayo en QuadSim
# ---------------------------------------------------------------------------
def _rc_norm_dz(pwm, mn, mx, trim, dz, reversed_=False):
    """RC_Channel::norm_input_dz."""
    pwm = np.asarray(pwm, dtype=float)
    lo, hi = trim - dz, trim + dz
    out = np.zeros_like(pwm)
    m1 = (pwm < lo) & (lo > mn)
    m2 = (pwm > hi) & (mx > hi)
    out[m1] = (pwm[m1] - lo) / (lo - mn)
    out[m2] = (pwm[m2] - hi) / (mx - hi)
    if reversed_:
        out = -out
    return np.clip(out, -1.0, 1.0)


def pilot_lean_angles(log: BinLog, t: np.ndarray, prm: ArduPilotParams = None
                      ) -> Tuple[np.ndarray, np.ndarray]:
    """
    Angulos que pide el piloto en Stabilize a partir de RCIN
    (Mode::get_pilot_desired_lean_angles_rad y rc_input_to_roll_pitch_rad).
    """
    prm = prm or params_from_log(log)
    P = log.params
    ch_r = int(P.get("RCMAP_ROLL", 1))
    ch_p = int(P.get("RCMAP_PITCH", 2))

    def norm(ch):
        pwm = log.at("RCIN", f"C{ch}", t)
        return _rc_norm_dz(pwm, P.get(f"RC{ch}_MIN", 1100.0), P.get(f"RC{ch}_MAX", 1900.0),
                           P.get(f"RC{ch}_TRIM", 1500.0), P.get(f"RC{ch}_DZ", 20.0),
                           bool(P.get(f"RC{ch}_REVERSED", 0)))
    rn, pn = norm(ch_r), norm(ch_p)
    amax = math.radians(min(prm.angle_max_deg, 85.0))
    tx = -np.tan(amax * pn)
    ty = np.tan(amax * rn)
    lim = math.tan(amax)
    ln = np.hypot(tx, ty)
    k = np.where(ln > lim, lim / np.maximum(ln, 1e-12), 1.0)
    tx, ty = tx * k, ty * k
    pitch = -np.arctan(tx)
    roll = np.arctan(np.cos(pitch) * ty)
    return roll, pitch


@dataclass
class SimReplay:
    t: np.ndarray
    real: Dict[str, np.ndarray]
    sim: Dict[str, np.ndarray]
    metrics: Dict[str, float]
    warnings: List[str]
    config: Dict[str, object]


def simulate_log(log: BinLog, vehicle: str = "arducopter_bbblue", t0: float = None,
                 t1: float = None, bench: Optional[BenchConfig] = None,
                 setpoints: str = "auto", seed: int = 1) -> SimReplay:
    """
    Repite en QuadSim el tramo [t0, t1] del registro. `bench` = None ->
    vuelo libre; un BenchConfig -> sobre la rotula (la actitud inicial se
    toma del registro). `setpoints`:
      "ang"  la actitud objetivo del firmware (ANG.Des*, ya moldeada por su
             modelo de comando) entra directo al controlador de actitud de
             QuadSim, con su derivada como prealimentacion. Compara solo
             controlador de actitud + PID + planta. Necesita ANG a la tasa
             del lazo (ATTITUDE_FAST).
      "rcin" las palancas del piloto (RCIN, 10 Hz) pasan por todo el camino
             de Stabilize, incluido el modelo de comando. El RCIN a 10 Hz
             deja hasta 0.1 s de incertidumbre en el instante de cada
             movimiento de palanca.
      "auto" "ang" si ANG esta a la tasa del lazo; si no, "rcin".
    """
    warns = list(log.log_bitmask_report())
    prm = params_from_log(log)
    if not log.has("ANG"):
        raise ValueError("el registro no tiene mensajes ANG (actitud del controlador)")
    if setpoints == "auto":
        setpoints = "ang" if log.rate_hz("ANG") >= 0.5 * prm.loop_rate_hz else "rcin"
    if setpoints == "rcin" and not log.has("RCIN"):
        warns.append("Sin RCIN: se usa la actitud objetivo ANG como referencia.")
        setpoints = "ang"
    ta = log.t("ANG")
    lo, hi = ta[0], ta[-1]
    if t0 is None:
        t0 = _first_armed(log, lo)
    if t1 is None:
        t1 = min(hi, t0 + 60.0)
    if not (lo <= t0 < t1 <= hi + 1e-9):
        raise ValueError(f"intervalo fuera del registro ({lo:.2f} a {hi:.2f} s)")
    cfg = SimConfig(vehicle=vehicle, ctrl_hz=float(prm.loop_rate_hz), attitude_law="ardupilot",
                    ardupilot=prm, seed=seed)
    if prm.gyro_filter_hz > 0 and prm.gyro_filter_hz < cfg.ctrl_hz / 2:
        for ax in AXES:
            cfg.gyro_filters[ax][0] = FilterStage("butterworth", float(prm.gyro_filter_hz))
    r0, p0, y0 = (float(log.at("ANG", f, [t0])[0]) for f in ("Roll", "Pitch", "Yaw"))
    if bench is not None:
        b = BenchConfig(**{k: getattr(bench, k) for k in bench.__dataclass_fields__})
        b.enabled = True
        b.initial_roll_deg, b.initial_pitch_deg, b.initial_yaw_deg = r0, p0, y0
        b.thrust_ratio = max(b.thrust_ratio, 1e-3)    # armado; el acelerador va aparte
        cfg.bench = b
    else:
        cfg.start_airborne = True
        cfg.start_alt = 1.5
    sim = Simulation(cfg)
    from quadsim import math3d as m3
    if bench is None:
        from quadsim.dynamics import IQ
        sim.x[IQ:IQ + 4] = list(m3.euler_to_q(*(math.radians(v) for v in (r0, p0, y0))))
        sim._prime_airborne()        # sensores y AHRS alineados con la actitud inicial
    dt = sim.dt_ctrl
    n = int(round((t1 - t0) / dt))
    tt = t0 + dt * np.arange(1, n + 1)
    # rumbo: la actitud objetivo del registro (en Stabilize el piloto manda
    # tasa de rumbo; la ley de QuadSim recibe el angulo que resulta)
    sy = np.interp(tt, ta, np.unwrap(np.radians(log.get("ANG", "DesYaw"))))
    targets = None
    if setpoints == "rcin":
        sr, sp_ = pilot_lean_angles(log, tt, prm)
    else:
        sr = np.radians(log.at("ANG", "DesRoll", tt, hold=False))
        sp_ = np.radians(log.at("ANG", "DesPitch", tt, hold=False))
        e_rate = [np.gradient(v, tt) for v in (sr, sp_, sy)]
        targets = (sr, sp_, sy, e_rate)
    thr = log.at("CTUN", "ThI", tt) if log.has("CTUN") else None
    if thr is None:
        warns.append("Sin CTUN: el acelerador del piloto se toma de RATE.AOut.")
        thr = log.at("RATE", "AOut", tt)
    sim.ardupilot_throttle_in = float(thr[0])
    sim.ap_ctrl.prime(m3.euler_to_q(*sim.est_att), float(thr[0]))
    keys = ("roll", "pitch", "yaw", "p", "q", "r", "rout", "pout", "yout",
            "m1", "m2", "m3", "m4")
    S = {k: np.empty(n) for k in keys}
    for i in range(n):
        sim.setpoint["roll"], sim.setpoint["pitch"] = float(sr[i]), float(sp_[i])
        sim.setpoint["yaw"] = m3.wrap_pi(float(sy[i]))
        sim.ardupilot_throttle_in = float(thr[i])
        if targets is not None:
            ph, th, ps = float(sr[i]), float(sp_[i]), float(sy[i])
            q_t = m3.euler_to_q(ph, th, ps)
            er = (e_rate[0][i], e_rate[1][i], e_rate[2][i])
            w_t = (er[0] - math.sin(th) * er[2],
                   math.cos(ph) * er[1] + math.sin(ph) * math.cos(th) * er[2],
                   -math.sin(ph) * er[1] + math.cos(th) * math.cos(ph) * er[2])
            sim.ardupilot_target = (q_t, w_t)
        sim.step_control()
        e = sim.est_att
        S["roll"][i], S["pitch"][i] = math.degrees(e[0]), math.degrees(e[1])
        S["yaw"][i] = math.degrees(e[2]) % 360.0
        g = sim.meas_rates
        S["p"][i], S["q"][i], S["r"][i] = (math.degrees(v) for v in g)
        o = sim.ap_ctrl.rpy
        S["rout"][i], S["pout"][i], S["yout"][i] = o
        pw = [sim.ap_ctrl.pwm(a) for a in sim.sigma]
        S["m1"][i], S["m2"][i], S["m3"][i], S["m4"][i] = pw
    R = {"roll": log.at("ANG", "Roll", tt, hold=False),
         "pitch": log.at("ANG", "Pitch", tt, hold=False),
         "yaw": log.at("ANG", "Yaw", tt, hold=False)}
    if log.has("RATE"):
        for k, f in (("p", "R"), ("q", "P"), ("r", "Y"), ("rout", "ROut"), ("pout", "POut"),
                     ("yout", "YOut")):
            R[k] = log.at("RATE", f, tt, hold=False)
    if log.has("RCOU"):
        for j in range(4):
            R[f"m{j + 1}"] = log.at("RCOU", f"C{j + 1}", tt)
    met = {}
    for k in ("roll", "pitch", "p", "q", "rout", "pout"):
        if k in R:
            d = S[k] - R[k]
            met[f"rms_{k}"] = float(np.sqrt(np.mean(d ** 2)))
    if "yaw" in R:
        d = (S["yaw"] - R["yaw"] + 180.0) % 360.0 - 180.0
        met["rms_yaw"] = float(np.sqrt(np.mean(d ** 2)))
    conf = {"vehiculo": vehicle, "desde_s": float(t0), "hasta_s": float(t1),
            "banco": bench is not None, "referencias": setpoints,
            "lazo_hz": cfg.ctrl_hz, "parametros": prm.source}
    return SimReplay(tt, R, S, met, warns, conf)


def _first_armed(log: BinLog, default: float) -> float:
    if log.has("CTUN"):
        t, thi = log.t("CTUN"), log.get("CTUN", "ThI")
        i = np.nonzero(thi > 0.05)[0]
        if len(i):
            return max(float(t[i[0]]), log.t("ANG")[0])
    return float(default)


# ---------------------------------------------------------------------------
# salida: CSV, figura y resumen
# ---------------------------------------------------------------------------
def write_report(rep: SimReplay, pids: List[PIDReplay], outdir: str,
                 title: str = "") -> List[str]:
    os.makedirs(outdir, exist_ok=True)
    files = []
    # CSV
    cols = ["t"]
    data = [rep.t]
    for k in rep.sim:
        if k in rep.real:
            cols += [f"{k}_real", f"{k}_sim"]
            data += [rep.real[k], rep.sim[k]]
    path = os.path.join(outdir, "comparacion.csv")
    np.savetxt(path, np.column_stack(data), delimiter=",", header=",".join(cols),
               comments="", fmt="%.6g")
    files.append(path)
    # resumen
    lines = [f"Comparacion sim-vs-real{': ' + title if title else ''}", ""]
    for k, v in rep.config.items():
        lines.append(f"  {k}: {v}")
    lines.append("")
    lines.append("Error cuadratico medio, simulador menos dron:")
    units = {"roll": "deg", "pitch": "deg", "yaw": "deg", "p": "deg/s", "q": "deg/s",
             "rout": "", "pout": ""}
    for k, v in rep.metrics.items():
        u = units.get(k.replace("rms_", ""), "")
        lines.append(f"  {k}: {v:.4g} {u}")
    if pids:
        lines.append("")
        lines.append("PID de tasa de QuadSim alimentado con el registro (RATE.RDes, RATE.R):")
        for r in pids:
            lines.append(f"  {r.axis}: RMS {r.rms:.4g}, maximo {r.max_abs:.4g} "
                         f"(salida en [-1, 1], RATE a {r.rate_hz:.0f} Hz)")
    ws = list(rep.warnings) + [w for r in pids for w in r.warnings]
    if ws:
        lines.append("")
        lines.append("Avisos:")
        lines += [f"  - {w}" for w in dict.fromkeys(ws)]
    path = os.path.join(outdir, "resumen.txt")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")
    files.append(path)
    # figura
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return files
    panels = [("roll", "Alabeo [°]"), ("pitch", "Cabeceo [°]"), ("p", "Tasa de alabeo [°/s]"),
              ("rout", "Salida PID roll"), ("m1", "Motor 1 [µs]")]
    panels = [p for p in panels if p[0] in rep.real]
    fig, axs = plt.subplots(len(panels), 1, figsize=(10, 2.2 * len(panels)), sharex=True)
    axs = np.atleast_1d(axs)
    for ax, (k, lab) in zip(axs, panels):
        ax.plot(rep.t, rep.real[k], color="#444444", lw=1.0, label="dron")
        ax.plot(rep.t, rep.sim[k], color="#d0582a", lw=1.0, label="QuadSim")
        ax.set_ylabel(lab)
        ax.grid(alpha=0.3)
    axs[0].legend(loc="upper right")
    axs[-1].set_xlabel("Tiempo del registro [s]")
    if title:
        axs[0].set_title(title)
    fig.tight_layout()
    path = os.path.join(outdir, "comparacion.png")
    fig.savefig(path, dpi=120)
    plt.close(fig)
    files.append(path)
    if pids:
        fig, axs = plt.subplots(len(pids), 1, figsize=(10, 2.2 * len(pids)), sharex=True)
        axs = np.atleast_1d(axs)
        for ax, r in zip(axs, pids):
            ax.plot(r.t, r.real, color="#444444", lw=1.0, label="firmware (RATE)")
            ax.plot(r.t, r.model, color="#2a7fd0", lw=1.0, ls="--", label="PID de QuadSim")
            ax.set_ylabel(f"Salida {r.axis}")
            ax.grid(alpha=0.3)
        axs[0].legend(loc="upper right")
        axs[-1].set_xlabel("Tiempo del registro [s]")
        fig.tight_layout()
        path = os.path.join(outdir, "pid_repetido.png")
        fig.savefig(path, dpi=120)
        plt.close(fig)
        files.append(path)
    return files
