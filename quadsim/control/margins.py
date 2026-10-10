"""
quadsim.control.margins
=======================
Margenes de estabilidad del lazo de TASA (por eje) en el dominio de la
frecuencia, con la cadena configurada: PID discreto (I por Euler hacia
atras, D sobre la medida con su filtro PT1), filtros del giroscopio
(biquads), retardo de la IMU, retencion del control (ZOH a ctrl_hz),
retardo de computo, muestreo y retencion del ESC con retardo de pulso,
motor de primer orden y eje como integrador (linealizacion en hover, ver
control/tuning.py):

    L(jw) = C(z) F(z) * e^{-jw d_imu} * H_zoh(T) * z^{-n_comp}
            * H_zoh(1/f_esc) e^{-jw d_pulso} * K / (jw (T_m jw + 1)),
    z = e^{jw T},  H_zoh(h) = (1 - e^{-jw h}) / (jw h)

Sirve para cuantificar lo que la sintonia SIMC "agrupa" en theta: el margen
de fase en el cruce y la ganancia maxima estable (multiplicador comun de
Kp, Ki y Kd = margen de ganancia lineal). Es un modelo LINEAL alrededor del
hover; los numeros absolutos dependen de ese supuesto, las tendencias (cuanto
se pierde con cada muestra de retardo o con un ESC lento) son lo que importa.
"""

from __future__ import annotations

import math
from typing import Dict, Iterable, Optional

import numpy as np

from ..filters import Biquad


def _stages(filters) -> list:
    out = []
    for st in filters or ():
        if hasattr(st, "ftype"):
            st = (st.ftype, st.fc, st.q, st.gain_db)
        out.append(tuple(st))
    return out


def rate_loop_L(f_hz: np.ndarray, K: float, Tm: float, ctrl_dt: float,
                kp: float, ki: float, kd: float, d_alpha: float,
                filters: Iterable = (), imu_delay: float = 0.0,
                compute_delay_samples: int = 0, esc_rate_hz: Optional[float] = None,
                esc_pulse_delay_s: float = 0.0) -> np.ndarray:
    """Respuesta en frecuencia del lazo abierto (complejo), f_hz arreglo."""
    f = np.asarray(f_hz, float)
    w = 2.0 * np.pi * f
    jw = 1j * w
    T = ctrl_dt
    zi = np.exp(-jw * T)                                   # z^-1
    C = kp + ki * T / (1.0 - zi) + kd * (1.0 - zi) / T * d_alpha / (1.0 - (1.0 - d_alpha) * zi)
    F = np.ones_like(jw)
    fs = 1.0 / T
    for ftype, fc, q, gdb in _stages(filters):
        if ftype == "none":
            continue
        b = Biquad(fs, ftype, fc, q, gdb)
        F = F * (b.b0 + b.b1 * zi + b.b2 * zi * zi) / (1.0 + b.a1 * zi + b.a2 * zi * zi)

    def zoh(h):
        return (1.0 - np.exp(-jw * h)) / (jw * h)

    D = np.exp(-jw * imu_delay) * zoh(T) * zi ** int(compute_delay_samples)
    if esc_rate_hz:
        D = D * zoh(1.0 / esc_rate_hz) * np.exp(-jw * esc_pulse_delay_s)
    P = K / (jw * (Tm * jw + 1.0))
    return C * F * D * P


def margins(L: np.ndarray, f_hz: np.ndarray) -> Dict[str, float]:
    """Cruce de ganancia, margen de fase, cruce de fase y margen de ganancia
    (interpolacion lineal en la malla)."""
    f = np.asarray(f_hz, float)
    mag = np.abs(L)
    ph = np.degrees(np.unwrap(np.angle(L)))
    out = {"f_cross_hz": math.nan, "phase_margin_deg": math.nan,
           "f_180_hz": math.nan, "gain_margin_db": math.inf, "max_gain_factor": math.inf}
    i = np.nonzero((mag[:-1] >= 1.0) & (mag[1:] < 1.0))[0]
    if len(i):
        k = i[0]
        a = (math.log(mag[k]) - 0.0) / (math.log(mag[k]) - math.log(mag[k + 1]))
        out["f_cross_hz"] = float(f[k] + a * (f[k + 1] - f[k]))
        out["phase_margin_deg"] = float(180.0 + ph[k] + a * (ph[k + 1] - ph[k]))
    j = np.nonzero((ph[:-1] > -180.0) & (ph[1:] <= -180.0))[0]
    if len(j):
        k = j[0]
        a = (ph[k] + 180.0) / (ph[k] - ph[k + 1])
        m = math.exp(math.log(mag[k]) + a * (math.log(mag[k + 1]) - math.log(mag[k])))
        out["f_180_hz"] = float(f[k] + a * (f[k + 1] - f[k]))
        out["gain_margin_db"] = float(-20.0 * math.log10(m))
        out["max_gain_factor"] = float(1.0 / m)
    return out


def sim_rate_loop_margins(sim, axis: str = "roll", **override) -> Dict[str, float]:
    """
    Margenes del lazo de tasa de `axis` con la configuracion y las ganancias
    ACTUALES de una Simulation (vehiculo nominal). `override` reemplaza
    cualquier argumento de rate_loop_L (p. ej. compute_delay_samples=1).
    """
    c = sim.cfg
    p = sim.nominal
    pid = sim.att_ctrl.rate_pid[axis]
    kw = dict(K=p.max_torque(axis) / p.inertia(axis), Tm=p.Tm, ctrl_dt=sim.dt_ctrl,
              kp=pid.g.kp, ki=pid.g.ki, kd=pid.g.kd, d_alpha=pid._alpha,
              filters=c.gyro_filters.get(axis, ()), imu_delay=c.sensors.imu_delay_s,
              compute_delay_samples=c.compute_delay_samples, esc_rate_hz=c.esc_rate_hz,
              esc_pulse_delay_s=c.esc_pulse_delay_s)
    kw.update(override)
    f = np.logspace(-1, math.log10(0.499 / sim.dt_ctrl), 6000)
    return margins(rate_loop_L(f, **kw), f)
