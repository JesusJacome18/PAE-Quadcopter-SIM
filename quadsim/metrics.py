"""
quadsim.metrics
===============
Metricas de desempeno sobre el diccionario que devuelve
`Simulation.log_arrays()` (claves t, pos_i, ref_i, vel_i, u_norm_i,
att_sp_i, euler_i, mixer_k_rp, thrust, ...). Todas usan la posicion
VERDADERA (pos_i), no la estimada: lo que importa en un articulo es donde
estuvo el dron, no donde creyo estar.

RESPUESTA AL ESCALON (por eje, definiciones clasicas; Ogata, "Modern
Control Engineering", 5a ed., sec. 5-3):
    rise_time      tiempo de subida 10 % -> 90 % del escalon
    overshoot_pct  sobreimpulso maximo en % del tamano del escalon
    settling_time  tiempo hasta quedar DEFINITIVAMENTE dentro de +-2 % del
                   escalon (medido desde el instante del escalon)
    ss_error       error de estado estacionario: media del error en el
                   ultimo 10 % del tramo (o al menos 0.5 s)
  Nota: si el escalon es pequeno, la banda del 2 % puede caer por debajo
  del ruido del sensor de posicion (1 cm): settling_time = nan en ese caso
  se reporta asi, no se inventa.

SEGUIMIENTO (error e(t) = p(t) - p_ref(t)):
    rmse_x, rmse_y, rmse_z, rmse_3d   raiz del error cuadratico medio
    max_err_3d                        maximo de ||e||
    iae                               integral de ||e|| dt
    itae                              integral de t ||e|| dt (t desde t_start)
    effort_u                          RMS de ||u_norm|| (momento normalizado
                                      del lazo de actitud, [-1,1]^3): el MISMO
                                      actuador para los 5 controladores, asi
                                      que es una medida justa del esfuerzo
    tilt_sp_rms_deg                   RMS de la inclinacion de referencia
    thrust_std                        desviacion estandar del empuje [N]
    sat_pct                           % del tiempo con el mezclador
                                      desaturando (mixer_k_rp < 1): el
                                      controlador pidio mas momento del que
                                      los motores pueden dar
"""

from __future__ import annotations

import math
from typing import Dict, Optional

import numpy as np

# reexportado para que la GUI y los experimentos usen una sola fuente
from .scenarios import prepare_tracking_sim  # noqa: F401


def _trapz(y, x):
    return float(np.sum(0.5 * (y[1:] + y[:-1]) * np.diff(x))) if len(x) > 1 else 0.0


def step_metrics(t: np.ndarray, y: np.ndarray, t_step: float, y0: float, y1: float,
                 t_end: Optional[float] = None, band: float = 0.02) -> Dict[str, float]:
    """Metricas de UN escalon y0 -> y1 aplicado en t_step (senal y(t))."""
    t = np.asarray(t, float)
    y = np.asarray(y, float)
    t_end = t[-1] if t_end is None else t_end
    m = (t >= t_step) & (t <= t_end)
    ts, ys = t[m] - t_step, y[m]
    d = y1 - y0
    out = {"step": d, "rise_time": math.nan, "overshoot_pct": math.nan,
           "settling_time": math.nan, "ss_error": math.nan}
    if len(ts) < 3 or abs(d) < 1e-9:
        return out
    yn = (ys - y0) / d                     # normalizada: 0 -> 1
    i10 = np.argmax(yn >= 0.1) if np.any(yn >= 0.1) else None
    i90 = np.argmax(yn >= 0.9) if np.any(yn >= 0.9) else None
    if i10 is not None and i90 is not None:
        out["rise_time"] = float(ts[i90] - ts[i10])
    out["overshoot_pct"] = float(max(0.0, (np.max(yn) - 1.0) * 100.0))
    outside = np.abs(yn - 1.0) > band
    if not outside[-1]:
        idx = np.where(outside)[0]
        out["settling_time"] = float(ts[idx[-1] + 1]) if len(idx) else 0.0
    n_last = int(np.sum(ts >= ts[-1] - 0.5))
    n_tail = max(int(0.1 * len(ts)), n_last, 1)
    out["ss_error"] = float(np.mean(y1 - ys[-n_tail:]))
    return out


def step_metrics_log(L: Dict[str, np.ndarray], traj, guard: float = 1.0) -> list:
    """
    Metricas de todos los escalones de una StepWaypoints: una entrada por
    (escalon, eje que cambia). Devuelve lista de dicts con 'k', 'axis'.

    guard: cada tramo se evalua hasta `guard` segundos ANTES del siguiente
    escalon. Motivo: un MPC con prevision de horizonte H (20 x 0.05 s = 1 s)
    empieza a moverse hacia el proximo escalon antes de que ocurra; sin
    esta guarda, ese movimiento anticipado se contaria como error de estado
    estacionario del tramo actual (se observo: 10 cm "falsos" en z). La
    misma ventana se aplica a TODOS los controladores.
    """
    out = []
    t = L["t"]
    steps = traj.steps()
    for k, (ts, p_from, p_to, t_end) in enumerate(steps):
        if k + 1 < len(steps):
            t_end = t_end - guard
        for ax in range(3):
            if abs(p_to[ax] - p_from[ax]) > 1e-9:
                r = step_metrics(t, L[f"pos_{ax}"], ts, p_from[ax], p_to[ax], t_end)
                r.update({"k": k, "axis": "xyz"[ax]})
                out.append(r)
    return out


def step_summary(L, traj) -> Dict[str, float]:
    """Promedios de las metricas de escalon (horizontal y vertical aparte)."""
    rows = step_metrics_log(L, traj)
    res = {}
    for grp, axes in (("xy", "xy"), ("z", "z")):
        sel = [r for r in rows if r["axis"] in axes]
        for key in ("rise_time", "overshoot_pct", "settling_time", "ss_error"):
            vals = np.array([abs(r[key]) if key == "ss_error" else r[key] for r in sel])
            res[f"{key}_{grp}"] = float(np.nanmean(vals)) if len(vals) and \
                np.any(np.isfinite(vals)) else math.nan
    return res


def tracking_metrics(L: Dict[str, np.ndarray], t_start: float = 0.0,
                     t_end: Optional[float] = None) -> Dict[str, float]:
    """Metricas de seguimiento entre t_start y t_end (por defecto todo)."""
    t = np.asarray(L["t"], float)
    t_end = t[-1] if t_end is None else t_end
    m = (t >= t_start) & (t <= t_end)
    tt = t[m]
    e = np.stack([L[f"pos_{i}"][m] - L[f"ref_{i}"][m] for i in range(3)], 1)
    en = np.linalg.norm(e, axis=1)
    out = {f"rmse_{a}": float(np.sqrt(np.mean(e[:, i] ** 2))) for i, a in enumerate("xyz")}
    out["rmse_3d"] = float(np.sqrt(np.mean(en ** 2)))
    out["max_err_3d"] = float(np.max(en))
    out["iae"] = _trapz(en, tt)
    out["itae"] = _trapz((tt - tt[0]) * en, tt)
    if "u_norm_0" in L:
        u = np.stack([L[f"u_norm_{i}"][m] for i in range(3)], 1)
        out["effort_u"] = float(np.sqrt(np.mean(np.sum(u ** 2, axis=1))))
    if "att_sp_0" in L:
        r = np.radians(L["att_sp_0"][m])
        p = np.radians(L["att_sp_1"][m])
        tilt = np.degrees(np.arccos(np.clip(np.cos(r) * np.cos(p), -1, 1)))
        out["tilt_sp_rms_deg"] = float(np.sqrt(np.mean(tilt ** 2)))
    if "thrust" in L:
        out["thrust_std"] = float(np.std(L["thrust"][m]))
    if "mixer_k_rp" in L:
        out["sat_pct"] = float(100.0 * np.mean(L["mixer_k_rp"][m] < 0.999))
    out["duration"] = float(tt[-1] - tt[0]) if len(tt) else 0.0
    return out
